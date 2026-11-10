function SerialLogRecorder
% SERIALLOGRECORDER Giao dien ghi log Serial vao file TXT.
% Chay trong MATLAB bang lenh: SerialLogRecorder

    app = struct();
    app.serial = [];
    app.fileId = -1;
    app.isRecording = false;
    app.startedAt = [];
    app.displayRows = cell(0, 2);
    app.maxDisplayRows = 5000; % Chi gioi han dong hien thi, KHONG gioi han file.

    app.fig = uifigure('Name', 'Serial Log Recorder', ...
        'Position', [100 100 900 620], ...
        'Color', [0.96 0.97 0.98]);
    app.fig.CloseRequestFcn = @closeApp;

    layout = uigridlayout(app.fig, [4 1]);
    layout.RowHeight = {128, 48, 28, '1x'};
    layout.Padding = [16 16 16 16];
    layout.RowSpacing = 10;

    settings = uipanel(layout, 'Title', 'Cau hinh Serial va file log');
    g = uigridlayout(settings, [3 4]);
    g.RowHeight = {28, 28, 28};
    g.ColumnWidth = {110, '1x', 110, '1x'};
    g.Padding = [10 8 10 8];

    uilabel(g, 'Text', 'Cong Serial:');
    app.portField = uidropdown(g, 'Items', availablePorts(), 'Editable', 'on', ...
        'ValueChangedFcn', @configurationChanged);
    uilabel(g, 'Text', 'Baud rate:');
    app.baudField = uieditfield(g, 'numeric', 'Value', 115200, ...
        'Limits', [1 Inf], 'RoundFractionalValues', 'on', ...
        'ValueChangedFcn', @configurationChanged);

    uilabel(g, 'Text', 'Ten file .txt:');
    app.fileNameField = uieditfield(g, 'text', 'Value', 'serial_log.txt');
    uilabel(g, 'Text', 'Thu muc luu:');
    app.folderField = uieditfield(g, 'text', 'Value', pwd);

    uilabel(g, 'Text', 'Ket thuc dong:');
    app.terminatorField = uidropdown(g, 'Items', {'LF', 'CR/LF', 'CR'}, ...
        'Value', 'LF', 'ValueChangedFcn', @configurationChanged);
    uilabel(g, 'Text', 'Trang thai:');
    app.statusLabel = uilabel(g, 'Text', 'San sang', 'FontWeight', 'bold', ...
        'FontColor', [0.2 0.4 0.2]);

    controls = uigridlayout(layout, [1 5]);
    controls.ColumnWidth = {150, 150, 100, 120, '1x'};
    app.startButton = uibutton(controls, 'push', 'Text', 'Bat dau ghi file', ...
        'FontWeight', 'bold', 'ButtonPushedFcn', @startRecording);
    app.stopButton = uibutton(controls, 'push', 'Text', 'Dung ghi file', ...
        'Enable', 'off', 'ButtonPushedFcn', @stopRecording);
    uilabel(controls, 'Text', 'Da ghi:');
    app.elapsedLabel = uilabel(controls, 'Text', '0 giay', ...
        'FontSize', 16, 'FontWeight', 'bold');
    app.pathLabel = uilabel(controls, 'Text', '', 'FontAngle', 'italic');

    app.hint = uilabel(layout, 'Text', ...
        'Log duoc ghi lien tuc vao file. Bang chi giu 5.000 dong gan nhat de giao dien luon muot.', ...
        'FontColor', [0.35 0.35 0.35]);

    app.table = uitable(layout, 'ColumnName', {'Thoi gian', 'Serial log'}, ...
        'ColumnWidth', {180, 'auto'}, 'Data', app.displayRows, ...
        'RowName', []);

    app.timer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.2, ...
        'TimerFcn', @updateElapsed);

    function ports = availablePorts()
        try
            ports = serialportlist("available");
            ports = cellstr(ports);
        catch
            ports = {};
        end
        if isempty(ports)
            ports = {''};
        end
    end

    function configurationChanged(~, ~)
        if app.isRecording
            return
        end
    end

    function startRecording(~, ~)
        if app.isRecording
            return
        end
        port = strtrim(app.portField.Value);
        fileName = strtrim(app.fileNameField.Value);
        folder = strtrim(app.folderField.Value);
        if isempty(port)
            uialert(app.fig, 'Hay nhap hoac chon cong Serial (vi du: COM3).', 'Thieu cong Serial');
            return
        end
        if isempty(fileName)
            uialert(app.fig, 'Hay nhap ten file log.', 'Thieu ten file');
            return
        end
        if ~endsWith(lower(fileName), '.txt')
            fileName = [fileName '.txt'];
            app.fileNameField.Value = fileName;
        end
        if isempty(folder)
            uialert(app.fig, 'Hay nhap duong dan thu muc luu file.', 'Thieu duong dan');
            return
        end
        if ~isfolder(folder)
            try
                mkdir(folder);
            catch ME
                uialert(app.fig, ME.message, 'Khong tao duoc thu muc');
                return
            end
        end

        fullName = fullfile(folder, fileName);
        try
            app.fileId = fopen(fullName, 'a', 'n', 'UTF-8');
            if app.fileId == -1
                error('Khong the mo file de ghi.');
            end
            app.serial = serialport(port, app.baudField.Value, 'Timeout', 2);
            configureTerminator(app.serial, app.terminatorField.Value);
            flush(app.serial);
            configureCallback(app.serial, 'terminator', @onSerialLine);
        catch ME
            closeResources();
            uialert(app.fig, ME.message, 'Khong the bat dau');
            return
        end

        app.isRecording = true;
        app.startedAt = datetime('now');
        app.startButton.Enable = 'off';
        app.stopButton.Enable = 'on';
        app.statusLabel.Text = 'Dang ghi';
        app.statusLabel.FontColor = [0 0.45 0.1];
        app.pathLabel.Text = fullName;
        start(app.timer);
    end

    function onSerialLine(src, ~)
        if ~app.isRecording || app.fileId == -1
            return
        end
        try
            line = readline(src);
            timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS.FFF');
            % Moi dong nhan duoc duoc luu ngay lap tuc, khong co gioi han kich thuoc file.
            fprintf(app.fileId, '[%s] %s\n', timestamp, line);
            app.displayRows(end+1, :) = {timestamp, char(line)};
            if size(app.displayRows, 1) > app.maxDisplayRows
                app.displayRows(1, :) = [];
            end
            app.table.Data = app.displayRows;
            drawnow limitrate nocallbacks
        catch ME
            if app.isRecording
                app.statusLabel.Text = ['Loi doc Serial: ' ME.message];
                app.statusLabel.FontColor = [0.75 0 0];
            end
        end
    end

    function updateElapsed(~, ~)
        if app.isRecording
            secondsElapsed = floor(seconds(datetime('now') - app.startedAt));
            app.elapsedLabel.Text = sprintf('%d giay', secondsElapsed);
        end
    end

    function stopRecording(varargin)
        if ~app.isRecording
            return
        end
        app.isRecording = false;
        if isvalid(app.timer) && strcmp(app.timer.Running, 'on')
            stop(app.timer);
        end
        closeResources();
        app.startButton.Enable = 'on';
        app.stopButton.Enable = 'off';
        app.statusLabel.Text = 'Da dung ghi';
        app.statusLabel.FontColor = [0.25 0.25 0.25];
    end

    function closeResources()
        if ~isempty(app.serial)
            try
                configureCallback(app.serial, 'off');
                delete(app.serial);
            catch
            end
            app.serial = [];
        end
        if app.fileId ~= -1
            try
                fclose(app.fileId);
            catch
            end
            app.fileId = -1;
        end
    end

    function closeApp(~, ~)
        stopRecording();
        if isvalid(app.timer)
            delete(app.timer);
        end
        delete(app.fig);
    end
end
