%% ========================================================================
%  SISO CP-OFDM Receiver - Multimedia (Dinamik Header + FEC + TXT Dosya)
%  UTF-8 Destekli, Bayt Seviyesinde Frame Yönetimi ve ÇOĞUNLUK OYLAMASI
%  =======================================================================
clear; clc; close all;

params = OFDM_define_params();

%% --- 1. Klasör ve Veri Kaydı Hazırlığı ---
timestamp = datestr(now, 'dd.mm.yyyy_HH.MM');
save_dir = fullfile('Received_Data', timestamp);
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

fprintf('====== OFDM MULTİMEDYA SİSTEMİ BAŞLADI ======\n');
fprintf('Kayıt Klasörü: %s\n\n', save_dir);

%% --- 2. Ground Truth (Kusursuz Referanslar) ---
% Metin Dosyası Ground Truth Hazırlığı
try
    fid = fopen('iletilecek_metin.txt', 'r', 'n', 'UTF-8');
    if fid == -1, error('Dosya acilamadi'); end
    expected_txt_msg = fread(fid, '*char')';
    fclose(fid);
catch
    expected_txt_msg = 'Varsayilan test metni. Lutfen iletilecek_metin.txt dosyasi olusturun.';
end

% UTF-8 formatında hedef veriyi hesapla
txt_bytes_all_gt = unicode2native(expected_txt_msg, 'UTF-8');
max_bytes_per_frame = floor(params.payload_bits / 8);
total_txt_frames = ceil(length(txt_bytes_all_gt) / max_bytes_per_frame);

% Resim Ground Truth Hazırlığı
try
    gt_img = imread('ADSB_PHY.png'); 
    gt_img = imresize(gt_img, params.img_size);
    if size(gt_img, 3) == 1, gt_img = cat(3, gt_img, gt_img, gt_img); end
    gt_bin = de2bi(gt_img(:), 8, 'left-msb'); 
    full_img_bitstream = reshape(gt_bin.', [], 1); 
    
    total_needed_bits = params.total_img_frames * params.payload_bits;
    if length(full_img_bitstream) < total_needed_bits
        full_img_bitstream = [full_img_bitstream; zeros(total_needed_bits - length(full_img_bitstream), 1)];
    end
catch
    warning('Referans resim bulunamadı! Hata oranları yanlış çıkabilir.');
end

%% --- 3. Analiz Değişkenleri ---
Target_Frames = 2000;
perf_frame_counter = 0;
snr_range = 0:5:35; 
total_errors = zeros(size(snr_range));
total_bits_count = 0;
ber_results = zeros(size(snr_range));

img_buffer = zeros(params.total_img_frames * params.payload_bits, 1); 
received_img_frames = []; 

global_hw_errors = 0;
global_hw_bits   = 0;

% Alınan metinleri birleştirmek için dinamik tampon alan
rx_text_buffer = repmat({'[Eksik/Bozuk Paket]'}, total_txt_frames, 1);
rx_byte_history = cell(total_txt_frames, 1); % Çoğunluk oylaması için geçmiş bayt hafızası
received_txt_frames = [];

%% --- 4. SDRu Kurulumu ---
rxPlat   = 'B200';      
rxSerial = '31C9574';   

radio = comm.SDRuReceiver('Platform', rxPlat, 'SerialNum', rxSerial, ...
    'MasterClockRate', params.MasterClockRate, 'CenterFrequency', params.rfTxFreq, ...
    'Gain', params.RxGain, 'DecimationFactor', params.MasterClockRate / params.Fs, ...
    'SamplesPerFrame', 4 * params.unitFrame, 'OutputDataType', 'double');
radio.EnableBurstMode = true; radio.NumFramesInBurst = 1;

%% Arayüz Grafik Objelerinin İlklendirilmesi ---
hFig = figure('Name', 'SISO CP-OFDM Real-Time Monitor Dashboard', 'Position', [50 100 1550 520], 'Color', [0.06 0.06 0.06]);
set(hFig, 'InvertHardcopy', 'off'); 

subplot(1,3,1);
ideal_pts = qammod(0:params.M_mod-1, params.M_mod, 'UnitAveragePower', true);
plot(real(ideal_pts), imag(ideal_pts), 'o', 'MarkerSize', 16, 'MarkerFaceColor', [1 0.9 0], 'MarkerEdgeColor', 'k', 'LineWidth', 2); 
hold on; grid on;
hConstPlot = plot(NaN, NaN, 'r.', 'MarkerSize', 6); 
xlim([-2 2]); ylim([-2 2]); axis square; 
set(gca, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual', ...
         'DataAspectRatioMode', 'manual', 'PlotBoxAspectRatioMode', 'manual', ...
         'Color', [0.11 0.11 0.11], 'XColor', 'w', 'YColor', 'w', ...
         'GridColor', [0.35 0.35 0.35], 'GridAlpha', 0.6);
title('Takımyıldızı Diyagramı (Phase Tracked)', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('In-Phase (I)', 'Color', [0.8 0.8 0.8]); ylabel('Quadrature (Q)', 'Color', [0.8 0.8 0.8]);

subplot(1,3,2);
hBerPlot = semilogy(snr_range, zeros(size(snr_range)), 'b-s', 'LineWidth', 2, 'MarkerFaceColor', 'b', 'MarkerSize', 5);
grid on; ylim([1e-5 1]); xlim([0 35]);
title('BER vs SNR Performansı', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('SNR (dB)', 'Color', [0.8 0.8 0.8]); ylabel('Bit Error Rate (BER)', 'Color', [0.8 0.8 0.8]);
set(gca, 'Color', [0.11 0.11 0.11], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.35 0.35 0.35], 'GridAlpha', 0.6);

subplot(1,3,3);
hImgPlot = imshow(uint8(zeros([params.img_size, params.img_channels])));
title('Alınan Multimedya Görseli', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
set(gca, 'XColor', [0.2 0.2 0.2], 'YColor', [0.2 0.2 0.2]);

%% --- 5. Ana Alıcı Döngüsü ---
while true
    [rxSig, len, ~] = radio();
    if len <= 0, continue; end
    
    sigRMS = sqrt(mean(abs(rxSig).^2));
    mfOut = abs(conv(rxSig, params.matchedFilter));
    [maxMF, bestPeak] = max(mfOut(1:length(rxSig)));
    if maxMF < 0.40 * sigRMS * params.pLen, continue; end
    
    preambleStart = bestPeak - params.pLen + 1;
    if preambleStart < 1 || bestPeak > length(rxSig), continue; end
    
    rxPreamble = rxSig(preambleStart:bestPeak);
    phaseDiff  = angle(sum(rxPreamble(params.halfLen+1:2*params.halfLen) .* conj(params.preamble_h2)) * ...
                 conj(sum(rxPreamble(1:params.halfLen) .* conj(params.preamble_h1))));
    cfo_est    = phaseDiff * params.Fs / (2 * pi * params.halfLen);
    t_vec      = (0:length(rxSig)-1).' / params.Fs;
    rxSig_corr = rxSig .* exp(-1j * 2 * pi * cfo_est * t_vec);
    
    best_offset = 0; best_h = 0;
    for offset = -3:3
        frameStart = bestPeak + params.Nzp + 1 + offset;
        if frameStart + params.ofdmLen - 1 > length(rxSig_corr), continue; end
        temp_Y = OFDM_demodulation(params.N, params.M, params.Ncp, rxSig_corr(frameStart : frameStart + params.ofdmLen - 1));
        p_energy = mean(abs(temp_Y(params.dataRows, params.np)));
        if p_energy > best_h, best_h = p_energy; best_offset = offset; end
    end
    if best_h == 0, continue; end
    
    frameStart = bestPeak + params.Nzp + 1 + best_offset;
    clean_tempOFDM = rxSig_corr(frameStart : frameStart + params.ofdmLen - 1);

    %% --- Demodülasyon ve Phase Tracking ---
    clean_Y = OFDM_demodulation(params.N, params.M, params.Ncp, clean_tempOFDM);
    H_est_hw = clean_Y(params.dataRows, params.np) / (params.pilotValue/sqrt(params.Md));
    Y_eq_hw = clean_Y(params.dataRows, params.dataSymCols) ./ H_est_hw;
    
    for col = 1:size(Y_eq_hw, 2)
        sym_hw = Y_eq_hw(:, col);
        decs = qammod(qamdemod(sym_hw, params.M_mod, 'UnitAveragePower', true), params.M_mod, 'UnitAveragePower', true);
        phase_err = angle(mean(sym_hw .* conj(decs)));
        Y_eq_hw(:, col) = sym_hw .* exp(-1j * phase_err);
    end

    llr_hw = qamdemod(Y_eq_hw(:), params.M_mod, 'OutputType', 'approxllr', 'UnitAveragePower', true);
    data_bits = vitdec(llr_hw, params.trellis, params.tblen, 'trunc', 'unquant');
    
    header_bits  = data_bits(1:params.header_bits);
    payload_type = bi2de(header_bits(1:8)', 'left-msb');
    rx_frame_id  = bi2de(header_bits(9:24)', 'left-msb');
    payload_bits = data_bits(params.header_bits+1:end);
    
    is_valid_payload = false; 
    
    if payload_type == 0 
        % METİN DOSYASI İŞLEME (ÇOĞUNLUK OYLAMASI İLE)
        L_bytes = bi2de(header_bits(25:32)', 'left-msb'); 
        
        if L_bytes > 0 && (L_bytes * 8) <= params.payload_bits && rx_frame_id > 0
            
            % Eğer gelen ID mevcut tampondan büyükse tamponları genişlet
            if rx_frame_id > length(rx_text_buffer)
                for buf_idx = length(rx_text_buffer)+1 : rx_frame_id
                    rx_text_buffer{buf_idx} = '[Eksik/Bozuk Paket]';
                    rx_byte_history{buf_idx} = []; % Hafızayı ilklendir
                end
            end
            
            payload_end = L_bytes * 8;
            rx_matrix = reshape(payload_bits(1:payload_end), 8, []).';
            rx_bytes = bi2de(rx_matrix, 'left-msb')'; % Yatay vektör (Row vector) yapıyoruz
            
            % --- ÇOĞUNLUK OYLAMASI (MAJORITY VOTING) BAŞLANGICI ---
            if isempty(rx_byte_history{rx_frame_id})
                % İlk defa geliyorsa doğrudan kaydet
                rx_byte_history{rx_frame_id} = rx_bytes;
            elseif length(rx_bytes) == size(rx_byte_history{rx_frame_id}, 2)
                % Daha önce geldiyse ve paket uzunluğu doğruysa alt alta ekle
                rx_byte_history{rx_frame_id} = [rx_byte_history{rx_frame_id}; rx_bytes];
            end
            
            % Sütun bazında en çok tekrar eden baytları seç (Gürültü filtreleniyor)
            voted_bytes = mode(rx_byte_history{rx_frame_id}, 1);
            
            % Filtrelenmiş (oylanmış) kusursuz baytları karaktere dönüştür
            rx_chars_final = native2unicode(voted_bytes, 'UTF-8');
            % -------------------------------------------------------
            
            rx_text_buffer{rx_frame_id} = rx_chars_final;
            if ~ismember(rx_frame_id, received_txt_frames)
                received_txt_frames = [received_txt_frames, rx_frame_id];
            end
        else
            rx_chars_final = 'BOS/HATALI METIN';
        end
        
        % --- GROUND TRUTH OLUŞTURMA ---
        payload_gt = zeros(params.payload_bits, 1);
        if rx_frame_id > 0 && rx_frame_id <= total_txt_frames
            start_byte = (rx_frame_id - 1) * max_bytes_per_frame + 1;
            end_byte   = min(rx_frame_id * max_bytes_per_frame, length(txt_bytes_all_gt));
            gt_chunk_bytes = txt_bytes_all_gt(start_byte:end_byte);
            
            gt_bin  = de2bi(gt_chunk_bytes, 8, 'left-msb');
            gt_base = reshape(gt_bin.', [], 1);
            
            min_len = min(length(gt_base), params.payload_bits);
            payload_gt(1:min_len) = gt_base(1:min_len);
            
            rng(1911 + rx_frame_id); % TX ile eşleşen PRBS
            bits_remaining = params.payload_bits - length(gt_base);
            if bits_remaining > 0
                payload_gt(min_len + 1 : end) = randi([0 1], bits_remaining, 1); 
            end
        else
            rng(1911 + rx_frame_id);
            payload_gt = randi([0 1], params.payload_bits, 1);
        end
        
        is_valid_payload = true;
        
    elseif payload_type == 1 && rx_frame_id > 0 && rx_frame_id <= params.total_img_frames
        % RESİM İŞLEME
        start_idx = (rx_frame_id - 1) * params.payload_bits + 1;
        end_idx   = rx_frame_id * params.payload_bits;
        
        img_buffer(start_idx:end_idx) = payload_bits;
        if ~ismember(rx_frame_id, received_img_frames)
            received_img_frames = [received_img_frames, rx_frame_id]; 
        end
        payload_gt = full_img_bitstream(start_idx:end_idx); 
        is_valid_payload = true;
    end

    %% --- Hata Analizi ---
    if is_valid_payload
        perf_frame_counter = perf_frame_counter + 1;
        
        hw_frame_errs = sum(payload_gt ~= payload_bits);
        hw_frame_ber  = hw_frame_errs / params.payload_bits;
        global_hw_errors = global_hw_errors + hw_frame_errs;
        global_hw_bits   = global_hw_bits + params.payload_bits;
        
        if payload_type == 0
            print_text = rx_chars_final;
            if length(print_text) > 15, print_text = [print_text(1:15), '...']; end
            
            % Konsola kaçıncı oylamanın yapıldığını da yazdıralım
            vote_count = size(rx_byte_history{rx_frame_id}, 1);
            fprintf('TXT Frame BER: %.2e | ID: %d (Oylama: %d) | %s\n', ...
                     hw_frame_ber, rx_frame_id, vote_count, print_text);
        end

        for s_idx = 1:length(snr_range)
            rx_noisy = awgn(clean_tempOFDM, snr_range(s_idx), 'measured');
            Y_noisy = OFDM_demodulation(params.N, params.M, params.Ncp, rx_noisy);
            H_noisy = Y_noisy(params.dataRows, params.np) / (params.pilotValue/sqrt(params.Md));
            Y_eq_noisy = Y_noisy(params.dataRows, params.dataSymCols) ./ H_noisy;
            
            for col = 1:size(Y_eq_noisy, 2)
                sym_n = Y_eq_noisy(:, col);
                decs_n = qammod(qamdemod(sym_n, params.M_mod, 'UnitAveragePower', true), params.M_mod, 'UnitAveragePower', true);
                phase_err_n = angle(mean(sym_n .* conj(decs_n)));
                Y_eq_noisy(:, col) = sym_n .* exp(-1j * phase_err_n);
            end
            
            llr_noisy = qamdemod(Y_eq_noisy(:), params.M_mod, 'OutputType', 'approxllr', 'UnitAveragePower', true);
            bits_noisy = vitdec(llr_noisy, params.trellis, params.tblen, 'trunc', 'unquant');
            
            total_errors(s_idx) = total_errors(s_idx) + sum(payload_gt ~= bits_noisy(params.header_bits+1:end));
        end
        total_bits_count = total_bits_count + params.payload_bits;
        ber_results = total_errors / total_bits_count;
    end

    %% --- Akıllı Görselleştirme Döngüsü ---
    if mod(perf_frame_counter, 10) == 0
        if isvalid(hFig) && ~isempty(received_img_frames)
            set(hConstPlot, 'XData', real(Y_eq_hw(:)), 'YData', imag(Y_eq_hw(:)));
            set(hBerPlot, 'YData', ber_results);
            title(subplot(1,3,2), sprintf('BER vs SNR (%d/%d)', perf_frame_counter, Target_Frames), 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
            
            rx_matrix = reshape(img_buffer(1:params.total_img_bits), 8, []).';
            rx_img_data = reshape(uint8(bi2de(rx_matrix, 'left-msb')), [params.img_size, params.img_channels]);
            set(hImgPlot, 'CData', rx_img_data);
            title(subplot(1,3,3), sprintf('Görüntü: %d / %d Paket', length(received_img_frames), params.total_img_frames), 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
            
            drawnow;
        end
    end

    %% --- Dinamik Durdurma ve Nihai Kayıt ---
    if (perf_frame_counter >= Target_Frames) && (length(received_img_frames) >= params.total_img_frames)
        fprintf('\n====== ANALİZ TAMAMLANDI. KAYITLAR YAPILIYOR... ======\n');
        
        final_hw_ber = global_hw_errors / global_hw_bits;
        
        report_path = fullfile(save_dir, 'Simulation_Report.txt');
        fid = fopen(report_path, 'w');
        fprintf(fid, '====== OFDM SİSTEM PERFORMANS RAPORU ======\n');
        fprintf(fid, 'Tarih/Saat: %s\n', timestamp);
        fprintf(fid, 'Toplam Analiz Edilen Frame: %d\n', perf_frame_counter);
        fprintf(fid, 'Toplam İletilen Bit (Donanım): %d\n', global_hw_bits);
        fprintf(fid, 'Toplam Hatalı Bit (Donanım)   : %d\n', global_hw_errors);
        fprintf(fid, 'Ortalama BER         : %.2e\n', final_hw_ber);
        fprintf(fid, '\nSNR (dB) | BER (Sanal AWGN)\n%s\n', repmat('-', 1, 25));
        for i = 1:length(snr_range)
            fprintf(fid, '%7d | %.2e\n', snr_range(i), ber_results(i));
        end
        fclose(fid);
        
        % Tamamlanan metin dosyasını eksiksiz sırasıyla birleştir ve kaydet ---
        text_path = fullfile(save_dir, 'Received_Text_File.txt');
        fid_txt = fopen(text_path, 'w', 'n', 'UTF-8');
        for idx_txt = 1:length(rx_text_buffer)
            fprintf(fid_txt, '%s', rx_text_buffer{idx_txt});
        end
        fclose(fid_txt);
        % ---------------------------------------------------------------------
        
        save(fullfile(save_dir, 'Simulation_Data.mat'), 'snr_range', 'ber_results', 'global_hw_errors', 'global_hw_bits', 'final_hw_ber');
        imwrite(rx_img_data, fullfile(save_dir, 'Received_Image.png'));
        saveas(hFig, fullfile(save_dir, 'Performance_Dashboard.png'));
        
        fprintf('Tüm veriler kaydedildi: %s\n', save_dir);
        break; 
    end
end