plot(ROCCurveEMD(:,2), ROCCurveEMD(:,1));
hold on;
plot(ROCCurveCos(:,2), ROCCurveCos(:,1));
plot(ROCCurveAbs(:,2), ROCCurveAbs(:,1));
legend('EMD','Cos','Abs');


% plot(PRCurveEMD(:,2), PRCurveEMD(:,1));
% hold on;
% plot(PRCurveCos(:,2), PRCurveCos(:,1));
% plot(PRCurveAbs(:,2), PRCurveAbs(:,1));
% legend('EMD','Cos','Abs');

