import React, { useState } from 'react';
import { Smartphone, ShieldAlert, Wifi, Compass, CheckCircle2, X, QrCode, ExternalLink } from 'lucide-react';

interface MobileTestGuideModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export const MobileTestGuideModal: React.FC<MobileTestGuideModalProps> = ({ isOpen, onClose }) => {
  const [activeTab, setActiveTab] = useState<'steps' | 'permissions' | 'troubleshooting'>('steps');

  if (!isOpen) return null;

  const currentHost = typeof window !== 'undefined' ? window.location.hostname : 'localhost';
  const isHttps = typeof window !== 'undefined' ? window.location.protocol === 'https:' : false;

  return (
    <div className="fixed inset-0 z-50 bg-black/60 backdrop-blur-xs flex items-center justify-center p-4">
      <div className="bg-white border border-slate-200 rounded-xl w-full max-w-md overflow-hidden shadow-2xl flex flex-col max-h-[90vh]">
        {/* Header */}
        <div className="bg-slate-900 text-white p-4 flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Smartphone className="w-5 h-5 text-blue-400" />
            <div>
              <h2 className="text-sm font-bold leading-tight">Physical Mobile Testing Guide</h2>
              <p className="text-[11px] text-slate-400">Real-time hardware sensor & AI validation</p>
            </div>
          </div>
          <button
            onClick={onClose}
            className="p-1 rounded-md text-slate-400 hover:text-white hover:bg-slate-800 transition-colors cursor-pointer"
          >
            <X className="w-5 h-5" />
          </button>
        </div>

        {/* Tab Navigation */}
        <div className="flex border-b border-slate-200 bg-slate-50 text-xs font-semibold text-slate-600">
          <button
            onClick={() => setActiveTab('steps')}
            className={`flex-1 py-2.5 px-3 border-b-2 text-center transition-colors cursor-pointer ${
              activeTab === 'steps' ? 'border-blue-600 text-blue-700 bg-white font-bold' : 'border-transparent hover:text-slate-900'
            }`}
          >
            1. Connection Setup
          </button>
          <button
            onClick={() => setActiveTab('permissions')}
            className={`flex-1 py-2.5 px-3 border-b-2 text-center transition-colors cursor-pointer ${
              activeTab === 'permissions' ? 'border-blue-600 text-blue-700 bg-white font-bold' : 'border-transparent hover:text-slate-900'
            }`}
          >
            2. Sensor Access
          </button>
          <button
            onClick={() => setActiveTab('troubleshooting')}
            className={`flex-1 py-2.5 px-3 border-b-2 text-center transition-colors cursor-pointer ${
              activeTab === 'troubleshooting' ? 'border-blue-600 text-blue-700 bg-white font-bold' : 'border-transparent hover:text-slate-900'
            }`}
          >
            3. SSL / Help
          </button>
        </div>

        {/* Modal Body Content */}
        <div className="p-4 space-y-4 overflow-y-auto text-xs leading-relaxed text-slate-700 flex-1">
          {activeTab === 'steps' && (
            <div className="space-y-3">
              <div className="bg-blue-50 border border-blue-200 rounded-lg p-3 space-y-1">
                <div className="font-bold text-blue-900 flex items-center gap-1.5">
                  <Wifi className="w-4 h-4 text-blue-600" />
                  <span>Connect Phone to Local WiFi</span>
                </div>
                <p className="text-blue-800 text-[11px]">
                  Your smartphone and host computer running Vite must be connected to the exact same WiFi network.
                </p>
              </div>

              <div className="space-y-2">
                <div className="font-bold text-slate-900 flex items-center gap-1.5">
                  <QrCode className="w-4 h-4 text-emerald-600" />
                  <span>Scan Terminal QR Code or Open Network URL</span>
                </div>
                <p>
                  Look at your host computer terminal window for the printed QR code, or type the local IP HTTPS URL:
                </p>
                <div className="bg-slate-900 text-emerald-400 font-mono p-2.5 rounded-md text-[11px] break-all select-all flex items-center justify-between">
                  <span>https://{currentHost}:5173</span>
                  <ExternalLink className="w-3.5 h-3.5 text-slate-400 flex-shrink-0 ml-2" />
                </div>
              </div>

              <div className="bg-amber-50 border border-amber-200 rounded-lg p-3 space-y-1">
                <div className="font-bold text-amber-900 flex items-center gap-1.5">
                  <ShieldAlert className="w-4 h-4 text-amber-600" />
                  <span>Accept Self-Signed Certificate Warning</span>
                </div>
                <p className="text-amber-800 text-[11px]">
                  When opening HTTPS on local IP, your mobile browser will show "Your connection is not private". Tap <strong>Advanced</strong> → <strong>Proceed to {currentHost} (unsafe)</strong>.
                </p>
              </div>
            </div>
          )}

          {activeTab === 'permissions' && (
            <div className="space-y-3">
              <div className="border border-slate-200 rounded-lg p-3 space-y-2">
                <div className="font-bold text-slate-900 flex items-center gap-1.5">
                  <Compass className="w-4 h-4 text-purple-600" />
                  <span>iOS Safari (iPhone / iPad)</span>
                </div>
                <ol className="list-decimal list-inside space-y-1 text-[11px] text-slate-600">
                  <li>Open <strong>Settings</strong> → <strong>Safari</strong> on iOS.</li>
                  <li>Ensure <strong>Motion & Orientation Access</strong> is toggled <strong>ON</strong>.</li>
                  <li>When requested by ReckonX, tap <strong>Allow</strong> for Motion and Location.</li>
                </ol>
              </div>

              <div className="border border-slate-200 rounded-lg p-3 space-y-2">
                <div className="font-bold text-slate-900 flex items-center gap-1.5">
                  <Compass className="w-4 h-4 text-blue-600" />
                  <span>Android Chrome</span>
                </div>
                <ol className="list-decimal list-inside space-y-1 text-[11px] text-slate-600">
                  <li>Tap lock/tune icon next to URL in Chrome.</li>
                  <li>Tap <strong>Permissions</strong> → Enable <strong>Sensors</strong> & <strong>Location</strong>.</li>
                  <li>Reload page if prompted.</li>
                </ol>
              </div>

              <div className="bg-emerald-50 border border-emerald-200 rounded-lg p-3 flex items-start gap-2 text-emerald-900">
                <CheckCircle2 className="w-4 h-4 text-emerald-600 flex-shrink-0 mt-0.5" />
                <div className="text-[11px]">
                  <strong>AI Verification:</strong> Walk 5–10 steps holding your device. Watch the top HUD badge transition from <span className="font-mono bg-slate-800 text-white px-1 py-0.5 rounded text-[10px]">AI ⋯</span> to <span className="font-mono bg-violet-700 text-white px-1 py-0.5 rounded text-[10px]">AI 95%</span>!
                </div>
              </div>
            </div>
          )}

          {activeTab === 'troubleshooting' && (
            <div className="space-y-3">
              <div className="space-y-1">
                <h4 className="font-bold text-slate-900">Why is HTTPS required for mobile testing?</h4>
                <p className="text-[11px] text-slate-600 leading-relaxed">
                  Modern mobile OS security models (iOS 13+ and Android 10+) strictly mandate secure origin contexts (<code className="bg-slate-100 px-1 py-0.5 rounded">https://</code> or <code className="bg-slate-100 px-1 py-0.5 rounded">localhost</code>) to access high-frequency motion APIs (<code className="bg-slate-100 px-1 py-0.5 rounded">DeviceMotionEvent</code>, <code className="bg-slate-100 px-1 py-0.5 rounded">DeviceOrientationEvent</code>).
                </p>
              </div>

              <div className="space-y-1">
                <h4 className="font-bold text-slate-900">Current Context Status:</h4>
                <div className="bg-slate-100 p-2.5 rounded-md font-mono text-[11px] space-y-1">
                  <div>Protocol: <span className={isHttps ? 'text-emerald-600 font-bold' : 'text-amber-600 font-bold'}>{isHttps ? 'HTTPS (Secure)' : 'HTTP (Insecure for remote mobile)'}</span></div>
                  <div>Host: <span className="text-slate-800">{currentHost}</span></div>
                </div>
              </div>

              <div className="bg-slate-50 border border-slate-200 p-3 rounded-lg space-y-1 text-[11px]">
                <div className="font-bold text-slate-900">Need Backend Sync on Phone?</div>
                <p>
                  Ensure Fastify backend is running (<code className="bg-slate-200 px-1 py-0.5 rounded">cd backend && npm run dev</code>). Your phone will stream session points to the local database automatically.
                </p>
              </div>
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="p-3 bg-slate-50 border-t border-slate-200 flex justify-end">
          <button
            onClick={onClose}
            className="px-4 py-2 bg-slate-900 hover:bg-slate-800 text-white font-bold rounded-lg text-xs transition-colors cursor-pointer"
          >
            Got it, close
          </button>
        </div>
      </div>
    </div>
  );
};
