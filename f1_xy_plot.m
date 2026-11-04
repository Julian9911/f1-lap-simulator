cx = out.carX_log.Data;  cy = out.carY_log.Data;
figure; plot(trackX, trackY, 'k'); hold on; axis equal; grid on
plot(cx, cy, 'r', 'LineWidth', 1.5);
err = arrayfun(@(i) min(hypot(trackX - cx(i), trackY - cy(i))), 1:numel(cx));
fprintf('Error lateral maximo: %.2f m\n', max(err));