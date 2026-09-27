const fs = require('fs');

const content = fs.readFileSync('e:\\reckonX_SIH\\idr2.csv', 'utf-8');
const lines = content.split('\n').map((l) => l.trim()).filter((l) => l.length > 0);

const imuWindow = [];
let zuptCount = 0;

for (let i = 1; i < lines.length; i++) {
    const cols = lines[i].split(',');
    if (cols.length < 10) continue;

    const accX = parseFloat(cols[1]);
    const accY = parseFloat(cols[2]);
    const accZ = parseFloat(cols[3]);
    const gyrX = parseFloat(cols[4]);
    const gyrY = parseFloat(cols[5]);
    const gyrZ = parseFloat(cols[6]);

    imuWindow.push([accX, accY, accZ, gyrX, gyrY, gyrZ]);
    if (imuWindow.length > 20) imuWindow.shift();

    if (imuWindow.length === 20) {
        let sumAx = 0, sumAy = 0, sumAz = 0;
        let sumGx = 0, sumGy = 0, sumGz = 0;
        for (const s of imuWindow) {
            sumAx += s[0]; sumAy += s[1]; sumAz += s[2];
            sumGx += s[3]; sumGy += s[4]; sumGz += s[5];
        }
        const n = 20;
        const meanAx = sumAx / n, meanAy = sumAy / n, meanAz = sumAz / n;
        const meanGx = sumGx / n, meanGy = sumGy / n, meanGz = sumGz / n;

        let varA = 0, varG = 0;
        for (const s of imuWindow) {
            varA += Math.pow(s[0] - meanAx, 2) + Math.pow(s[1] - meanAy, 2) + Math.pow(s[2] - meanAz, 2);
            varG += Math.pow(s[3] - meanGx, 2) + Math.pow(s[4] - meanGy, 2) + Math.pow(s[5] - meanGz, 2);
        }
        varA /= n;
        varG /= n;

        const isStationary = varA < 0.5 && varG < 0.05;
        if (isStationary) zuptCount++;
        if (i < 50) {
            console.log(`Row ${i}: varA=${varA.toFixed(4)}, varG=${varG.toFixed(4)}, ZUPT=${isStationary}`);
        }
    }
}
console.log(`Total rows: ${lines.length - 1}`);
console.log(`ZUPT triggered: ${zuptCount} times`);
