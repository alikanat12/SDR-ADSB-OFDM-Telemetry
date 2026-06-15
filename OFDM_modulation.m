function [s, ofdmLen] = OFDM_modulation(N, M, Ncp, X_TF)
%% OFDM Modulation: IFFT + CP Addition
%  OTFS_modulation fonksiyonuyla aynı mantık:
%    OTFS: x_DD  -> ISFFT -> Heisenberg -> s (zaman domeni)
%    OFDM: X_TF  -> IFFT  -> CP ekle    -> s (zaman domeni)
%
%  Giriş:
%    N    - OFDM sembol sayısı (= OTFS Doppler ekseni)
%    M    - FFT boyutu / alt taşıyıcı sayısı (= OTFS gecikme ekseni)
%    Ncp  - Cyclic prefix uzunluğu (her sembol için)
%    X_TF - M x N frekans domeni kaynak ızgarası
%           Her sütun bir OFDM sembolünün frekans domeni gösterimi
%
%  Çıkış:
%    s       - N*(M+Ncp) x 1 zaman domeni OFDM sinyali
%    ofdmLen - Toplam sinyal uzunluğu
%
%  Örnek:
%    X_TF = zeros(128, 16);
%    X_TF(2:end, :) = qpsk_symbols;  % DC null, diğerleri veri
%    s = OFDM_modulation(16, 128, 16, X_TF);

ofdmLen = N * (M + Ncp);
s = zeros(ofdmLen, 1);

for k = 1:N
    % IFFT: frekans -> zaman
    x_time = ifft(X_TF(:, k)) * sqrt(M);
    
    % CP ekle: son Ncp örneği başa kopyala
    cp = x_time(end - Ncp + 1 : end);
    ofdm_sym = [cp; x_time];  % (Ncp + M) x 1
    
    % Frame'e yerleştir
    idx = (k - 1) * (M + Ncp) + 1;
    s(idx : idx + M + Ncp - 1) = ofdm_sym;
end

end
