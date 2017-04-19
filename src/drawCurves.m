plot(ROCCurve05Cos(:, 2), ROCCurve05Cos(:, 1),'r');
hold on
plot(ROCCurve05EMD(:, 2), ROCCurve05EMD(:, 1),'b');
xlabel('FPR');ylabel('TPR');
legend('Cos','EMD');