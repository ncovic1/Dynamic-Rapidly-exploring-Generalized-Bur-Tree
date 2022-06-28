global graphics_WS graphics_CS graphics_local;

subplot(1,2,2);
title(' ');
for i = 1:length(graphics_CS)
    delete(graphics_CS{i});
end
for i = 1:length(graphics_WS)
    for j = 1 : length(graphics_WS{i})
        delete(graphics_WS{i}{j});
    end
end
for i = 1:length(graphics_local)
    for j = 1 : length(graphics_local{i})
        delete(graphics_local{i}{j});
    end
end