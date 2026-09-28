import { haversineDistance } from '../services/ekf/OutputStabilizer';
import type { FusedState } from '../services/ekf/FusionRuntime';

/**
 * Helper to compute distance in km between two [lat, lon] coordinates.
 */
function distanceKm(p1: [number, number], p2: [number, number]): number {
  return haversineDistance(p1[0], p1[1], p2[0], p2[1]) / 1000.0;
}

/**
 * Calculate the projection of a point onto a line segment [p1, p2].
 * Returns the nearest coordinate on the segment.
 */
export function projectPointOnSegment(
  p: [number, number],
  p1: [number, number],
  p2: [number, number]
): [number, number] {
  const [lat, lng] = p;
  const [lat1, lng1] = p1;
  const [lat2, lng2] = p2;

  const dx = lng2 - lng1;
  const dy = lat2 - lat1;

  if (dx === 0 && dy === 0) {
    return p1;
  }

  // Parameter t of projection onto line segment
  const t = Math.max(0, Math.min(1, ((lng - lng1) * dx + (lat - lat1) * dy) / (dx * dx + dy * dy)));

  return [lat1 + t * dy, lng1 + t * dx];
}

/**
 * Calculate the accurate remaining road distance along the polyline from current position.
 * Accepts either a [lat, lng] coordinate tuple or an EKF FusedState.
 * 1. Finds the closest segment on the polyline to the vehicle position.
 * 2. Projects vehicle onto that segment.
 * 3. Sums distance from projected point to segment end + remaining polyline segments.
 */
export function calculateRemainingRoadDistance(
  currentPos: [number, number] | FusedState,
  routeCoordinates: [number, number][]
): { remainingDistanceKm: number; nearestSegmentIndex: number; offRouteDistanceMeters: number } {
  if (!routeCoordinates || routeCoordinates.length === 0) {
    return { remainingDistanceKm: 0, nearestSegmentIndex: 0, offRouteDistanceMeters: 0 };
  }

  const pos: [number, number] = Array.isArray(currentPos)
    ? currentPos
    : [currentPos.latitude ?? 0, currentPos.longitude ?? 0];

  // Explicit guard: if position is uninitialized or at (0,0), return full route length and do not trigger arrival
  if (
    pos[0] === null || pos[0] === undefined || isNaN(pos[0]) ||
    pos[1] === null || pos[1] === undefined || isNaN(pos[1]) ||
    (Math.abs(pos[0]) < 0.0001 && Math.abs(pos[1]) < 0.0001)
  ) {
    let totalKm = 0;
    for (let i = 0; i < routeCoordinates.length - 1; i++) {
      totalKm += distanceKm(routeCoordinates[i], routeCoordinates[i + 1]);
    }
    return {
      remainingDistanceKm: Math.round(totalKm * 10) / 10,
      nearestSegmentIndex: 0,
      offRouteDistanceMeters: 0,
    };
  }

  if (routeCoordinates.length === 1) {
    const dist = distanceKm(pos, routeCoordinates[0]);
    return { remainingDistanceKm: dist, nearestSegmentIndex: 0, offRouteDistanceMeters: dist * 1000 };
  }

  let minDistanceToSegment = Infinity;
  let nearestSegmentIndex = 0;
  let nearestProjectedPoint: [number, number] = routeCoordinates[0];

  // Find the closest road segment along the route
  for (let i = 0; i < routeCoordinates.length - 1; i++) {
    const p1 = routeCoordinates[i];
    const p2 = routeCoordinates[i + 1];
    const projected = projectPointOnSegment(pos, p1, p2);
    const dist = distanceKm(pos, projected);

    if (dist < minDistanceToSegment) {
      minDistanceToSegment = dist;
      nearestSegmentIndex = i;
      nearestProjectedPoint = projected;
    }
  }

  // Distance from nearest projected point to the end of its segment
  const pNext = routeCoordinates[nearestSegmentIndex + 1];
  let remainingKm = distanceKm(nearestProjectedPoint, pNext);

  // Add all subsequent road segment lengths
  for (let i = nearestSegmentIndex + 1; i < routeCoordinates.length - 1; i++) {
    remainingKm += distanceKm(
      routeCoordinates[i],
      routeCoordinates[i + 1]
    );
  }

  return {
    remainingDistanceKm: Math.round(remainingKm * 10) / 10,
    nearestSegmentIndex,
    offRouteDistanceMeters: Math.round(minDistanceToSegment * 1000),
  };
}
