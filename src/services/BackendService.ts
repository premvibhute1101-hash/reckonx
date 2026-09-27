import type { RecordedGPSPoint, TrackingSessionPayload } from './api/trackingService';

export type BackendConnectionStatus = 'connected' | 'offline' | 'error';

type StatusChangeListener = (status: BackendConnectionStatus) => void;

class BackendService {
  private currentStatus: BackendConnectionStatus = 'offline';
  private listeners: Set<StatusChangeListener> = new Set();
  private healthCheckIntervalId: number | null = null;
  private isCheckingHealth = false;

  constructor() {
    // Read backend URL from env or fallback
    this.initHealthCheck();
  }

  private get backendUrl(): string {
    if (typeof import.meta !== 'undefined' && import.meta.env?.VITE_BACKEND_URL) {
      return import.meta.env.VITE_BACKEND_URL.replace(/\/$/, '');
    }
    if (typeof import.meta !== 'undefined' && import.meta.env?.VITE_API_BASE_URL) {
      return import.meta.env.VITE_API_BASE_URL.replace(/\/api\/?$/, '');
    }
    return 'http://localhost:3001';
  }

  /**
   * Subscribe to connection status changes
   */
  public subscribeStatus(listener: StatusChangeListener): () => void {
    this.listeners.add(listener);
    // Immediately emit current status to new subscriber
    listener(this.currentStatus);
    return () => {
      this.listeners.delete(listener);
    };
  }

  /**
   * Get current backend connection status
   */
  public getStatus(): BackendConnectionStatus {
    return this.currentStatus;
  }

  /**
   * Internal status updater - notifies subscribers on change
   */
  private updateStatus(newStatus: BackendConnectionStatus) {
    if (this.currentStatus !== newStatus) {
      this.currentStatus = newStatus;
      this.listeners.forEach((fn) => fn(newStatus));
    }
  }

  /**
   * Start periodic health check to monitor backend connectivity
   */
  public initHealthCheck() {
    this.checkHealth();
    if (!this.healthCheckIntervalId && typeof window !== 'undefined') {
      this.healthCheckIntervalId = window.setInterval(() => {
        this.checkHealth();
      }, 15000); // Check every 15s
    }
  }

  /**
   * Ping backend /health endpoint with timeout
   */
  public async checkHealth(): Promise<boolean> {
    if (this.isCheckingHealth) return this.currentStatus === 'connected';
    this.isCheckingHealth = true;

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 4000);

    try {
      const response = await fetch(`${this.backendUrl}/health`, {
        method: 'GET',
        headers: { Accept: 'application/json' },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        this.updateStatus('connected');
        this.isCheckingHealth = false;
        return true;
      } else {
        this.updateStatus('error');
        this.isCheckingHealth = false;
        return false;
      }
    } catch {
      clearTimeout(timeoutId);
      this.updateStatus('offline');
      this.isCheckingHealth = false;
      return false;
    }
  }

  /**
   * Stream batched telemetry points to backend (fails gracefully if offline)
   */
  public async syncTelemetryBatch(
    sessionId: string | undefined,
    points: RecordedGPSPoint[]
  ): Promise<{ success: boolean; pointsSaved: number }> {
    if (!points || points.length === 0) return { success: true, pointsSaved: 0 };

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 6000);

      const response = await fetch(`${this.backendUrl}/api/telemetry`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
        body: JSON.stringify({ sessionId, points }),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        this.updateStatus('connected');
        const data = await response.json();
        return { success: true, pointsSaved: data.pointsSaved || points.length };
      } else {
        console.warn(`[BackendService] Telemetry sync returned HTTP ${response.status}`);
        return { success: false, pointsSaved: 0 };
      }
    } catch (err: any) {
      console.warn('[BackendService] Telemetry sync failed (operating offline):', err.message || err);
      this.updateStatus('offline');
      return { success: false, pointsSaved: 0 };
    }
  }

  /**
   * Persist full completed session to backend
   */
  public async syncSession(
    payload: TrackingSessionPayload
  ): Promise<{ success: boolean; sessionId?: string }> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 10000);

      const response = await fetch(`${this.backendUrl}/api/sessions`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json', Accept: 'application/json' },
        body: JSON.stringify(payload),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        this.updateStatus('connected');
        const data = await response.json();
        return { success: true, sessionId: data.sessionId };
      } else {
        console.warn(`[BackendService] Session sync returned HTTP ${response.status}`);
        return { success: false };
      }
    } catch (err: any) {
      console.warn('[BackendService] Session sync failed (operating offline):', err.message || err);
      this.updateStatus('offline');
      return { success: false };
    }
  }

  /**
   * Fetch session from backend by ID
   */
  public async fetchSession(sessionId: string): Promise<any | null> {
    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 6000);

      const response = await fetch(`${this.backendUrl}/api/sessions/${sessionId}`, {
        headers: { Accept: 'application/json' },
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      if (response.ok) {
        this.updateStatus('connected');
        return await response.json();
      }
      return null;
    } catch (err: any) {
      console.warn(`[BackendService] Fetch session ${sessionId} failed:`, err.message || err);
      this.updateStatus('offline');
      return null;
    }
  }
}

export const backendService = new BackendService();
