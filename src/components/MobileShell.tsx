import React from 'react';

interface MobileShellProps {
  children: React.ReactNode;
  header?: React.ReactNode;
  footer?: React.ReactNode;
  hideHeaderPadding?: boolean;
  hideFooterPadding?: boolean;
}

export const MobileShell: React.FC<MobileShellProps> = ({
  children,
  header,
  footer,
  hideHeaderPadding = false,
  hideFooterPadding = false,
}) => {
  return (
    <div className="min-h-app-screen w-full bg-slate-100 flex items-center justify-center p-0 sm:p-4">
      <div className="w-full max-w-[430px] h-app-screen sm:h-[880px] bg-slate-50 relative flex flex-col overflow-hidden border-0 sm:border border-slate-300 sm:rounded-2xl shadow-none">
        {/* Top Docked Header Slot */}
        {header && (
          <div
            className="absolute top-0 left-0 right-0 w-full bg-white border-b border-slate-200 z-30 flex items-center px-4"
            style={{
              paddingTop: 'env(safe-area-inset-top, 0px)',
              height: 'calc(3.5rem + env(safe-area-inset-top, 0px))',
            }}
          >
            {header}
          </div>
        )}

        {/* Scrollable Body Area */}
        <div
          className={`flex-1 w-full overflow-y-auto relative z-10 mobile-scroll-container`}
          style={{
            paddingTop:
              header && !hideHeaderPadding
                ? 'calc(3.5rem + env(safe-area-inset-top, 0px))'
                : undefined,
            paddingBottom:
              footer && !hideFooterPadding
                ? 'calc(4rem + env(safe-area-inset-bottom, 0px) + 16px)'
                : undefined,
          }}
        >
          {children}
        </div>

        {/* Bottom Docked Navigation Slot */}
        {footer && (
          <div
            className="absolute bottom-0 left-0 right-0 w-full bg-white border-t border-slate-200 z-30 flex items-center justify-around"
            style={{
              height: 'calc(4rem + env(safe-area-inset-bottom, 0px))',
              paddingBottom: 'env(safe-area-inset-bottom, 0px)',
            }}
          >
            {footer}
          </div>
        )}
      </div>
    </div>
  );
};
