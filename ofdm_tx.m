%% ========================================================================
%  SISO CP-OFDM Transmitter - Multimedia (Metin Dosyası + Resim) + FEC
%  Dinamik Header + UTF-8 Destekli
%  ========================================================================
clear; clc; close all;

params = OFDM_define_params();

%% 1. Metin Dosyasını Hazırlama (.txt Okuma)
try
    fid = fopen('adsb_messages.txt', 'r', 'n', 'UTF-8');
    if fid == -1, error('Dosya acilamadi'); end
    txt_msg = fread(fid, '*char')';
    fclose(fid);
    fprintf('Metin dosyasi basariyla yüklendi.\n');
catch
    warning('iletilecek_metin.txt bulunamadı! Varsayılan metin kullanılıyor.');
    txt_msg = 'Varsayilan test metni. Lutfen iletilecek_metin.txt dosyasi olusturun.';
end

% UTF-8 karakter sorununu çözmek için metni doğrudan baytlara çeviriyoruz
txt_bytes_all = unicode2native(txt_msg, 'UTF-8');

% Metni frame'lere bölebilmek için maksimum BAYT kapasitesini hesapla
max_bytes_per_frame = floor(params.payload_bits / 8);
total_txt_frames = ceil(length(txt_bytes_all) / max_bytes_per_frame);
fprintf('Metin verisi toplam %d bayt, %d frame olarak iletilecek.\n', length(txt_bytes_all), total_txt_frames);

%% 2. Resim Verisini Hazırlama 
try
    img = imread('AirplaneE.png');
    img = imresize(img, params.img_size);
    if size(img, 3) == 1, img = cat(3, img, img, img); end
    img_bin = de2bi(img(:), 8, 'left-msb');
    img_bitstream = reshape(img_bin.', [], 1);
    
    total_needed_bits = params.total_img_frames * params.payload_bits;
    if length(img_bitstream) < total_needed_bits
        img_bitstream = [img_bitstream; zeros(total_needed_bits - length(img_bitstream), 1)];
    end
    
    fprintf('Resim yüklendi. Toplam %d frame olarak iletilecek.\n', params.total_img_frames);
catch
    error('Resim bulunamadı!');
end

%% 3. USRP Kurulumu
connectedRadios = findsdru;
if ~isempty(connectedRadios) && ~isempty(connectedRadios(1).Platform)
    txPlat = connectedRadios(1).Platform; 
    txSerial = connectedRadios(1).SerialNum;
else
    txPlat = 'B200'; 
    txSerial = '31C47B7'; 
end

radio = comm.SDRuTransmitter('Platform', txPlat, ...
    'SerialNum', txSerial, ...
    'MasterClockRate', params.MasterClockRate, ...
    'CenterFrequency', params.rfTxFreq, ...
    'Gain', params.USRPGain, ...
    'InterpolationFactor', params.MasterClockRate / params.Fs);

fprintf('=== OFDM Multimedya Yayını Başladı ===\nDurdurmak için Ctrl+C\n');

img_frame_idx = 1;
txt_frame_idx = 1; % Metin paketleri için sayaç

try
    while true
        % --- A. Metin Frame İletimi ---
        % İlgili metin bayt parçasını (chunk) seç
        start_byte = (txt_frame_idx - 1) * max_bytes_per_frame + 1;
        end_byte   = min(txt_frame_idx * max_bytes_per_frame, length(txt_bytes_all));
        chunk_bytes = txt_bytes_all(start_byte:end_byte);
        L_bytes    = length(chunk_bytes);
        
        txt_bin = de2bi(chunk_bytes, 8, 'left-msb');
        txt_base = reshape(txt_bin.', [], 1); 
        
        % Kalan boşluğu PRBS ile doldur
        bits_remaining = params.payload_bits - length(txt_base);
        rng(1911 + txt_frame_idx); % Her frame için eşsiz ama tutarlı seed
        prbs_pad = randi([0 1], bits_remaining, 1);
        
        txt_payload = [txt_base; prbs_pad];
        
        % HEADER [Tür (0), TXT_Frame_ID (16 bit), Uzunluk Bayt Cinsinden (8 bit)]
        header_txt = [de2bi(0, 8, 'left-msb'), de2bi(txt_frame_idx, 16, 'left-msb'), de2bi(L_bytes, 8, 'left-msb')].';
        frame_txt_bits = [header_txt; txt_payload];
        
        encoded_txt_bits = convenc(frame_txt_bits, params.trellis); % FEC Encoding
        x_txt = qammod(encoded_txt_bits, params.M_mod, 'InputType', 'bit', 'UnitAveragePower', true);
        X_TF_txt = zeros(params.M, params.N);
        X_TF_txt(params.dataRows, params.np) = params.pilotValue / sqrt(params.Md);
        X_TF_txt(params.dataRows, params.dataSymCols) = reshape(x_txt, params.Md, params.N - 1);
        [s_txt, ~] = OFDM_modulation(params.N, params.M, params.Ncp, X_TF_txt);
        s_txt = s_txt / max(abs(s_txt));
        TxWave_txt = [params.preamble; zeros(params.Nzp, 1); s_txt; s_txt] * 0.9;
        step(radio, TxWave_txt);
        
        txt_frame_idx = txt_frame_idx + 1;
        if txt_frame_idx > total_txt_frames, txt_frame_idx = 1; end
        
        % --- B. Resim Frame İletimi ---
        start_idx = (img_frame_idx - 1) * params.payload_bits + 1;
        end_idx   = img_frame_idx * params.payload_bits;
        
        img_payload = img_bitstream(start_idx:end_idx);
        header_img = [de2bi(1, 8, 'left-msb'), de2bi(img_frame_idx, 16, 'left-msb'), de2bi(0, 8, 'left-msb')].';
        frame_img_bits = [header_img; img_payload];
        
        encoded_img_bits = convenc(frame_img_bits, params.trellis); % FEC Encoding
        x_img = qammod(encoded_img_bits, params.M_mod, 'InputType', 'bit', 'UnitAveragePower', true);
        X_TF_img = zeros(params.M, params.N);
        X_TF_img(params.dataRows, params.np) = params.pilotValue / sqrt(params.Md);
        X_TF_img(params.dataRows, params.dataSymCols) = reshape(x_img, params.Md, params.N - 1);
        [s_img, ~] = OFDM_modulation(params.N, params.M, params.Ncp, X_TF_img);
        s_img = s_img / max(abs(s_img));
        TxWave_img = [params.preamble; zeros(params.Nzp, 1); s_img; s_img] * 0.9;
        step(radio, TxWave_img);
        
        img_frame_idx = img_frame_idx + 1;
        if img_frame_idx > params.total_img_frames, img_frame_idx = 1; end
    end
catch
    release(radio);
    fprintf('Yayın durduruldu.\n');
end