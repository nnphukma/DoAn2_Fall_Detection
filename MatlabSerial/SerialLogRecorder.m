function SerialLogRecorder
% SERIALLOGRECORDER Giao dien ghi log Serial vao file TXT.
% Chay trong MATLAB bang lenh: SerialLogRecorder

    app = struct();
    app.serial = [];
    app.fileId = -1;
    app.isRecording = false;
    app.startedAt = [];
    app.lineCount = 0;
    app.displayRows = cell(0, 2);
    app.pendingDisplayRows = cell(0, 2);
    app.displayUpdateEvery = 10;
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

    controls = uigridlayout(layout, [1 7]);
    controls.ColumnWidth = {150, 150, 68, 90, 70, 88, '1x'};
    app.startButton = uibutton(controls, 'push', 'Text', 'Bat dau ghi file', ...
        'FontWeight', 'bold', 'ButtonPushedFcn', @startRecording);
    app.stopButton = uibutton(controls, 'push', 'Text', 'Dung ghi file', ...
        'Enable', 'off', 'ButtonPushedFcn', @stopRecording);
    uilabel(controls, 'Text', 'Da ghi:');
    app.elapsedLabel = uilabel(controls, 'Text', '0 giay', ...
        'FontSize', 16, 'FontWeight', 'bold');
    uilabel(controls, 'Text', 'Da nhan:');
    app.lineCountLabel = uilabel(controls, 'Text', '0 dong', ...
        'FontSize', 16, 'FontWeight', 'bold');
    app.pathLabel = uilabel(controls, 'Text', '', 'FontAngle', 'italic');

    app.hint = uilabel(layout, 'Text', ...
        'Moi dong UART duoc ghi ngay vao file; bang chi lam moi sau 10 dong va giu 5.000 dong gan nhat.', ...
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

        try
            [app.fileId, fullName] = createNewLogFile(folder, fileName);
            app.serial = serialport(port, app.baudField.Value, 'Timeout', 2);
            configureTerminator(app.serial, app.terminatorField.Value);
            flush(app.serial);
        catch ME
            closeResources();
            uialert(app.fig, ME.message, 'Khong the bat dau');
            return
        end

        app.isRecording = true;
        app.startedAt = datetime('now');
        app.lineCount = 0;
        app.displayRows = cell(0, 2);
        app.pendingDisplayRows = cell(0, 2);
        app.table.Data = app.displayRows;
        app.elapsedLabel.Text = '0 giay';
        app.lineCountLabel.Text = '0 dong';
        app.startButton.Enable = 'off';
        app.stopButton.Enable = 'on';
        app.statusLabel.Text = 'Recording';
        app.statusLabel.FontColor = [0 0.45 0.1];
        app.pathLabel.Text = fullName;
        start(app.timer);
        % Bat callback sau khi trang thai va file da san sang de khong bo sot dong dau.
        try
            configureCallback(app.serial, 'terminator', @onSerialLine);
        catch ME
            stopRecording();
            uialert(app.fig, ME.message, 'Khong the bat dau nhan UART');
        end
    end

    function onSerialLine(src, ~)
        if ~app.isRecording || app.fileId == -1
            return
        end
        try
            line = readline(src);
            timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss.SSS'));
            % Ghi truoc, doc lap voi GUI: moi dong CSV da nhan deu duoc dua vao file.
            fprintf(app.fileId, '[%s] %s\n', timestamp, line);
            app.lineCount = app.lineCount + 1;
            app.pendingDisplayRows(end+1, :) = {timestamp, char(line)};

            % Khong cap nhat bang theo tung dong. GUI chi la phan xem, khong
            % tham gia vao dieu kien ghi file hay dem so mau.
            if mod(app.lineCount, app.displayUpdateEvery) == 0
                flushDisplay();
            end
        catch ME
            if app.isRecording
                handleRuntimeError(ME);
            end
        end
    end

    function flushDisplay()
        if isempty(app.pendingDisplayRows)
            return
        end
        app.displayRows = [app.displayRows; app.pendingDisplayRows]; %#ok<AGROW>
        app.pendingDisplayRows = cell(0, 2);
        if size(app.displayRows, 1) > app.maxDisplayRows
            app.displayRows = app.displayRows(end-app.maxDisplayRows+1:end, :);
        end
        app.table.Data = app.displayRows;
        app.lineCountLabel.Text = sprintf('%d dong', app.lineCount);
        drawnow limitrate nocallbacks
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
        % Hien thi phan du lieu cuoi cung (< 10 dong) truoc khi dong file.
        flushDisplay();
        closeResources();
        app.startButton.Enable = 'on';
        app.stopButton.Enable = 'off';
        app.statusLabel.Text = 'Stopped';
        app.statusLabel.FontColor = [0.25 0.25 0.25];
    end

    function handleRuntimeError(ME)
        % Dung an toan de file duoc dong ngay ca khi UART/ghi file loi.
        app.isRecording = false;
        if isvalid(app.timer) && strcmp(app.timer.Running, 'on')
            stop(app.timer);
        end
        flushDisplay();
        closeResources();
        app.startButton.Enable = 'on';
        app.stopButton.Enable = 'off';
        app.statusLabel.Text = ['Loi: ' ME.message];
        app.statusLabel.FontColor = [0.75 0 0];
    end

    function [fileId, fullName] = createNewLogFile(folder, requestedName)
        % Tao ten co timestamp; neu trung, them so thu tu. Khong ghi de file cu.
        [~, baseName, ext] = fileparts(requestedName);
        if isempty(ext)
            ext = '.txt';
        end
        stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
        sequence = 0;
        while true
            if sequence == 0
                candidate = sprintf('%s_%s%s', baseName, stamp, ext);
            else
                candidate = sprintf('%s_%s_%03d%s', baseName, stamp, sequence, ext);
            end
            fullName = fullfile(folder, candidate);
            if ~isfile(fullName)
                break
            end
            sequence = sequence + 1;
        end
        fileId = fopen(fullName, 'w', 'n', 'UTF-8');
        if fileId == -1
            error('Khong the tao file log moi: %s', fullName);
        end
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
