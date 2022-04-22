
function [extr, dextr, ddextr, eytr, deytr, ddeytr, extl, dextl, ddextl, eytl, deytl, ddeytl] = boundingboxderivatives(psi, v, w, l_r, a, B)

l=l_r;
extr = abs(w/2*cos(psi)-l/2*sin(psi));
dextr = -(B*v*conj(sign((w*cos(psi))/2 - (l*sin(psi))/2))*((cos(conj(psi))*conj(l))/2 + (sin(conj(psi))*conj(w))/2))/l_r;
ddextr = (B*v*((2*dirac((w*cos(psi))/2 - (l*sin(psi))/2)*conj(B)*conj(v)*((l*cos(psi))/2 + (w*sin(psi))/2)^2)/conj(l_r) - (sign((w*cos(psi))/2 - (l*sin(psi))/2)*conj(B)*conj(v)*((w*cos(psi))/2 - (l*sin(psi))/2))/conj(l_r)))/l_r - (a*sign((w*cos(psi))/2 - (l*sin(psi))/2)*conj(B)*((l*cos(psi))/2 + (w*sin(psi))/2))/conj(l_r);

eytr = abs((l*cos(psi))/2 + (w*sin(psi))/2);
deytr = (B*v*conj(sign((l*cos(psi))/2 + (w*sin(psi))/2))*((cos(conj(psi))*conj(w))/2 - (sin(conj(psi))*conj(l))/2))/l_r;
ddeytr = (B*v*((2*dirac((l*cos(psi))/2 + (w*sin(psi))/2)*conj(B)*conj(v)*((w*cos(psi))/2 - (l*sin(psi))/2)^2)/conj(l_r) - (sign((l*cos(psi))/2 + (w*sin(psi))/2)*conj(B)*conj(v)*((l*cos(psi))/2 + (w*sin(psi))/2))/conj(l_r)))/l_r + (a*sign((l*cos(psi))/2 + (w*sin(psi))/2)*conj(B)*((w*cos(psi))/2 - (l*sin(psi))/2))/conj(l_r);

extl = abs((w*cos(psi))/2 + (l*sin(psi))/2);
dextl = (B*v*conj(sign((w*cos(psi))/2 + (l*sin(psi))/2))*((cos(conj(psi))*conj(l))/2 - (sin(conj(psi))*conj(w))/2))/l_r;
ddextl = (B*v*((2*dirac((w*cos(psi))/2 + (l*sin(psi))/2)*conj(B)*conj(v)*((l*cos(psi))/2 - (w*sin(psi))/2)^2)/conj(l_r) - (sign((w*cos(psi))/2 + (l*sin(psi))/2)*conj(B)*conj(v)*((w*cos(psi))/2 + (l*sin(psi))/2))/conj(l_r)))/l_r + (a*sign((w*cos(psi))/2 + (l*sin(psi))/2)*conj(B)*((l*cos(psi))/2 - (w*sin(psi))/2))/conj(l_r);

eytl = abs((l*cos(psi))/2 - (w*sin(psi))/2);
deytl = -(B*v*conj(sign((l*cos(psi))/2 - (w*sin(psi))/2))*((cos(conj(psi))*conj(w))/2 + (sin(conj(psi))*conj(l))/2))/l_r;
ddeytl = (B*v*((2*dirac((l*cos(psi))/2 - (w*sin(psi))/2)*conj(B)*conj(v)*((w*cos(psi))/2 + (l*sin(psi))/2)^2)/conj(l_r) - (sign((l*cos(psi))/2 - (w*sin(psi))/2)*conj(B)*conj(v)*((l*cos(psi))/2 - (w*sin(psi))/2))/conj(l_r)))/l_r - (a*sign((l*cos(psi))/2 - (w*sin(psi))/2)*conj(B)*((w*cos(psi))/2 + (l*sin(psi))/2))/conj(l_r);