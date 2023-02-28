function Aj = computeMatrixAj_momentum(matrixOfJetsAxes, matrixOfJetsArms)

[r_J1, r_J2, r_J3, r_J4, ax_J1, ax_J2, ax_J3, ax_J4] = iRonCubLib.demuxJetAxesAndArms(matrixOfJetsAxes, matrixOfJetsArms);

% multiplier of jetsIntensities in the linear momentum equations
% (namely, the first 3 rows of Aj)
Aj_linear  = matrixOfJetsAxes;

% multiplier of jetsIntensities in the angular momentum equations
% (namely, the last 3 rows of Aj)
Aj_angular = [wbc.skew(r_J1) * ax_J1, ...
              wbc.skew(r_J2) * ax_J2, ...
              wbc.skew(r_J3) * ax_J3, ...
              wbc.skew(r_J4) * ax_J4];
               
% compute matrix Aj
Aj          = [Aj_linear; Aj_angular];

end