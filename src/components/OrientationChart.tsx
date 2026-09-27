import React, { useEffect, useRef } from 'react';
import { useNavigationContext } from '../context/NavigationContext';
import { Compass, AlertCircle } from 'lucide-react';

export const OrientationChart: React.FC = () => {
  const { telemetry, sensorStatus } = useNavigationContext();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const historyPitchRef = useRef<number[]>(new Array(100).fill(0));
  const historyRollRef = useRef<number[]>(new Array(100).fill(0));
  const historyYawRef = useRef<number[]>(new Array(100).fill(0));

  // Push actual orientation readings into history buffer when updated
  useEffect(() => {
    if (telemetry.isStreamingOrientation) {
      historyPitchRef.current.shift();
      historyPitchRef.current.push(telemetry.pitch);

      historyRollRef.current.shift();
      historyRollRef.current.push(telemetry.roll);

      historyYawRef.current.shift();
      historyYawRef.current.push(telemetry.yaw);
    }
  }, [telemetry.pitch, telemetry.roll, telemetry.yaw, telemetry.isStreamingOrientation]);

  // Render canvas frame
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animationFrameId: number;

    const render = () => {
      const width = canvas.width;
      const height = canvas.height;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#FFFFFF';
      ctx.fillRect(0, 0, width, height);

      // Light Gray Reference Grid
      ctx.strokeStyle = '#E2E8F0';
      ctx.lineWidth = 1;

      for (let y = 20; y < height; y += 30) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
      }

      for (let x = 40; x < width; x += 50) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
      }

      // Draw baseline zero line
      const zeroY = height / 2;
      ctx.strokeStyle = '#CBD5E1';
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(width, zeroY);
      ctx.stroke();
      ctx.setLineDash([]);

      const dataPitch = historyPitchRef.current;
      const dataRoll = historyRollRef.current;
      const dataYaw = historyYawRef.current;

      const drawLine = (data: number[], color: string, minVal: number, maxVal: number) => {
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;

        const step = width / (data.length - 1);
        data.forEach((val, index) => {
          const x = index * step;
          const normalized = (val - minVal) / (maxVal - minVal);
          const y = height - Math.max(0, Math.min(height, normalized * (height - 20) + 10));
          if (index === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.stroke();
      };

      // Plot pitch & roll (-180° to +180°) and yaw (0° to 360°)
      drawLine(dataYaw, '#64748B', 0, 360);
      drawLine(dataPitch, '#1D4ED8', -180, 180);
      drawLine(dataRoll, '#D97706', -180, 180);

      animationFrameId = requestAnimationFrame(render);
    };

    render();

    return () => {
      cancelAnimationFrame(animationFrameId);
    };
  }, []);

  const hasHardware = sensorStatus.hasOrientationHardware;
  const isStreaming = telemetry.isStreamingOrientation;

  return (
    <div className="w-full bg-white border border-slate-200 rounded-md p-3">
      <div className="flex items-center justify-between mb-2">
        <span className="text-xs font-bold text-slate-900 flex items-center gap-1.5">
          <Compass className="w-3.5 h-3.5 text-emerald-700" />
          <span>Real-time Orientation Waveform</span>
          <span className="text-[10px] text-slate-400 font-mono font-normal">(°)</span>
        </span>
        <div className="flex items-center gap-3 text-[10px] font-mono">
          <span className="flex items-center gap-1 text-blue-700">
            <span className="w-2 h-2 rounded-full bg-blue-700"></span> Pitch
          </span>
          <span className="flex items-center gap-1 text-amber-600">
            <span className="w-2 h-2 rounded-full bg-amber-600"></span> Roll
          </span>
          <span className="flex items-center gap-1 text-slate-500">
            <span className="w-2 h-2 rounded-full bg-slate-500"></span> Yaw
          </span>
        </div>
      </div>

      <div className="w-full h-32 bg-slate-50 border border-slate-200 rounded overflow-hidden relative">
        <canvas ref={canvasRef} width={400} height={128} className="w-full h-full block" />

        {/* Overlay State when orientation sensors are not streaming or unsupported */}
        {!isStreaming && (
          <div className="absolute inset-0 bg-white/85 backdrop-blur-2xs flex flex-col items-center justify-center p-3 text-center">
            {hasHardware ? (
              <>
                <div className="w-2.5 h-2.5 rounded-full bg-emerald-500 animate-ping mb-1.5" />
                <span className="text-xs font-bold text-slate-800">Waiting for orientation data...</span>
                <span className="text-[10px] text-slate-500 mt-0.5 font-sans">Tilt or rotate device to generate orientation events</span>
              </>
            ) : (
              <>
                <AlertCircle className="w-4 h-4 text-slate-400 mb-1" />
                <span className="text-xs font-bold text-slate-700">Orientation sensor not supported on this device</span>
                <span className="text-[10px] text-slate-400 mt-0.5 font-sans font-normal">Physical orientation sensor required for live waveform</span>
              </>
            )}
          </div>
        )}
      </div>
    </div>
  );
};
