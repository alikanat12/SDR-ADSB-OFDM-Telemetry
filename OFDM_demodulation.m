function Y_TF = OFDM_demodulation(N, M, Ncp, r)
%% OFDM Demodulation: CP Removal + FFT
%  OTFS_demodulation fonksiyonuyla aynı mantık:
%    OTFS: r (zaman) -> Wigner -> SFFT -> y_DD
%    OFDM: r (zaman) -> CP kaldır -> FFT -> Y_TF
%
%  Giriş:
%    N   - OFDM sembol sayısı
%    M   - FFT boyutu / alt taşıyıcı sayısı
%    Ncp - Cyclic prefix uzunluğu
%    r   - N*(M+Ncp) x 1 alınan zaman domeni sinyali
%
%  Çıkış:
%    Y_TF - M x N alınan frekans domeni kaynak ızgarası
%           Her sütun bir OFDM sembolünün frekans domeni gösterimi
%
%  Örnek:
%    Y_TF = OFDM_demodulation(16, 128, 16, rxSignal);
%    % Y_TF(2:end, dataSymIdx) -> eşitlenecek veri sembolleri

Y_TF = zeros(M, N);

for k = 1:N
    % CP kaldır: ilk Ncp örneği atla
    symStart = (k - 1) * (M + Ncp) + Ncp + 1;
    symEnd   = symStart + M - 1;
    
    if symEnd > length(r)
        warning('Sinyal kısa: sembol %d/%d atlanıyor', k, N);
        break;
    end
    
    y_time = r(symStart : symEnd);  % M x 1
    
    % FFT: zaman -> frekans
    Y_TF(:, k) = fft(y_time) / sqrt(M);
end

end
