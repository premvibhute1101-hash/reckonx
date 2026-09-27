import { EkfCore } from '../src/core/EkfCore';

const ekf = new EkfCore();

// Propagate a few times to build up P
for (let i=0; i<10; i++) {
    ekf.predict(0.1, [0,0,9.81], [0,0,0]);
}

// Intercept matrix operations to print R and K? No, we can just print the P before and after.
// Wait, I can't print R or K directly because they are local variables.
// But we can observe the partial snap back!

const posBefore = { ...ekf.getPosition() };
ekf.updateGnss([10, 10, 0], [0,0,0], 3.0, []);
const posAfter = { ...ekf.getPosition() };

console.log('Pos Before:', posBefore);
console.log('Pos GNSS:', {x: 10, y: 10, z: 0});
console.log('Pos After:', posAfter);

// To get exactly R and K, let's copy the update logic here:
import { Matrix, inverse } from 'ml-matrix';

const rScale = ekf.getGnssState().updateState(3.0);
console.log('rScale:', rScale);

let R = Matrix.eye(6).mul(1.0);
for (let i = 3; i < 6; i++) {
    R.set(i, i, 0.1); 
}
R = R.mulColumnVector(Matrix.columnVector(Array(6).fill(rScale)));
console.log('R diagonal (pos):', R.get(0,0));

// Wait, ml-matrix doesn't have mulColumnVector? Let's run it and see.
