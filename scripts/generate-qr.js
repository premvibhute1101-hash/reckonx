import os from 'os';
import qrcode from 'qrcode-terminal';

function getLocalIp() {
  const interfaces = os.networkInterfaces();
  for (const name of Object.keys(interfaces)) {
    for (const iface of interfaces[name] || []) {
      if (iface.family === 'IPv4' && !iface.internal) {
        return iface.address;
      }
    }
  }
  return 'localhost';
}

const localIp = getLocalIp();
const httpUrl = `http://${localIp}:5173`;
const httpsUrl = `https://${localIp}:5173`;
const backendUrl = `http://${localIp}:3001`;

console.log('\n======================================================');
console.log('      ReckonX Real-Time Physical Device Testing       ');
console.log('======================================================\n');
console.log(`📡 Local Network IP: ${localIp}`);
console.log(`🌐 HTTP Access:     ${httpUrl}`);
console.log(`🔒 HTTPS Access:    ${httpsUrl}`);
console.log(`🗄️ Backend API:     ${backendUrl}\n`);
console.log('📱 Scan this QR code on your mobile device (Same WiFi):\n');

qrcode.generate(httpsUrl, { small: true }, (qr) => {
  console.log(qr);
});

console.log('\n------------------------------------------------------');
console.log('⚠️  MOBILE DEVICE TESTING INSTRUCTIONS:');
console.log('  1. Mobile phone MUST be connected to the SAME WiFi network.');
console.log('  2. Scan QR code or open HTTPS URL above.');
console.log('  3. Accept self-signed SSL warning (Advanced -> Proceed).');
console.log('  4. Grant DeviceMotion & Orientation permissions when prompted.');
console.log('======================================================\n');
