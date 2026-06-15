function params = OFDM_define_params()
% OFDM_DEFINE_PARAMS  CP-OFDM sistemi (Metin + Resim + BER)

params.N   = 16;   
params.M   = 128;   
params.Ncp = 32;    

params.Mz = ceil(0.0666 * params.M);            
params.Md = 100;                                  
params.Mp = params.M - params.Md - params.Mz;    

params.dataRows  = params.Mz + 1 : params.Mz + params.Md;   
params.guardRows = [1:params.Mz, params.Mz+params.Md+1:params.M]; 

params.np         = 8;          
params.pilotValue = 10 + 10i;   
params.dataSymCols = setdiff(1:params.N, params.np);  

params.M_mod = 4;       
params.M_bits = log2(params.M_mod);

params.N_syms_perfram = (params.N - 1) * params.Md;          
params.N_bits_perfram = params.N_syms_perfram * params.M_bits; % 3000 bit

% MULTİMEDYA İLETİM & FEC PARAMETRELERİ ---
params.header_bits  = 32; 

% Kanal Kodlama (FEC) Parametreleri
params.trellis = poly2trellis(7, [171 133]); % 1/2 oranlı evrişimsel kodlayıcı
params.tblen   = 32;                         % Viterbi traceback derinliği
params.coding_rate = 1/2;

% 1/2 Kodlama sebebiyle frame içine sığacak ham bit sayısı yarıya düşer:
params.raw_bits_perfram = floor(params.N_bits_perfram * params.coding_rate);
params.payload_bits     = params.raw_bits_perfram - params.header_bits; 

% Resim Ayarları
params.img_size     = [64, 64];
params.img_channels = 3;
params.total_img_bits = prod(params.img_size) * params.img_channels * 8;
params.total_img_frames = ceil(params.total_img_bits / params.payload_bits);
% ----------------------------------------------

params.Nzp = 50;
params.preamble = zadoffChuSeq(25, 193);

params.rfTxFreq        = 2.5e9;   
params.MasterClockRate = 20e6;
params.Fs              = 2e6;
params.USRPGain        = 50;
params.RxGain          = 50;
params.InterpolationFactor = params.MasterClockRate / params.Fs;

params.ofdmLen   = params.N * (params.M + params.Ncp);
params.pLen      = length(params.preamble);
params.unitFrame = params.pLen + params.Nzp + 2 * params.ofdmLen;

params.matchedFilter = conj(params.preamble(end:-1:1));
params.halfLen       = floor(params.pLen / 2);
params.preamble_h1   = params.preamble(1 : params.halfLen);
params.preamble_h2   = params.preamble(params.halfLen + 1 : 2 * params.halfLen);
end