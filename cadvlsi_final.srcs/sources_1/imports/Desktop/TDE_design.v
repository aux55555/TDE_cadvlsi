`define CYCLE 10 // 單一cycle 長度

module TDE(
    input clk,
    input rst,
    input [9:0] x,  // 傳出波訊號 10 位元
    input [9:0] y,  // 接收波訊號 10 位元
    
    output reg busy,
    output reg valid,
    output reg [7:0] d,  // 距離輸出，仍然是 8 位元
    
    output [7:0] seg7,       // 七段顯示器段位輸出
    output [3:0] seg7_sel    // 七段顯示器位選
);
// 參數定義
parameter IDLE = 3'd0;
parameter INPUT = 3'd1;
parameter CALC = 3'd2;
parameter PEAK_SELECT = 3'd3;  // 新增峰值選擇狀態
parameter OUTPUT = 3'd4;

// 狀態與計數器
reg [2:0] state, next_state;
reg [7:0] cnt;  // 計算輸入了幾個數據
reg [7:0] calc_cnt;  // 計算k位移用
reg [7:0] max_k;  // 儲存最大R值對應的k

// 存儲空間
reg [9:0] x_buffer [0:149];  // 存儲x(t)，更新為 10 位元
reg [9:0] y_buffer [0:149];  // 存儲y(t)，更新為 10 位元
reg [31:0] R_max;  // 存儲最大的R值
reg [31:0] R_values [0:149];  // 存儲所有的R值用於插值

// 新增：用於存儲前一個樣本的暫存器
reg [9:0] x_prev;
reg [9:0] y_prev;

// 用於二次插值的暫存器
reg [31:0] R_before;  // 峰值前的R值
reg [31:0] R_peak;    // 峰值R值
reg [31:0] R_after;   // 峰值後的R值
reg [31:0] interp_num;   // 插值計算的分子
reg [31:0] interp_den;   // 插值計算的分母
reg [7:0] fine_offset;   // 精確偏移量

// 用於計算R值的暫存器
reg [31:0] R_temp;
reg [31:0] mul_temp;
integer i;

// 硬編碼數據
reg [9:0] x_hardcoded [0:149];
reg [9:0] y_hardcoded [0:149];

// 初始化硬編碼數據
initial begin
    for (i = 0; i < 150; i = i + 1) begin
        if (i >= 70 && i < 80)
            x_hardcoded[i] = 10'd200;  // 峰值
        else
            x_hardcoded[i] = 10'd50;   // 基準值
    end
    for (i = 0; i < 150; i = i + 1) begin
        if (i >= 100 && i < 110)
            y_hardcoded[i] = 10'd180;  // 峰值（衰減）
        else
            y_hardcoded[i] = 10'd50;   // 基準值
    end
end

// 狀態機 - 時序邏輯
always @(posedge clk or posedge rst) begin
    if(rst) begin
        state <= IDLE;
    end
    else begin
        state <= next_state;
    end
end

// 狀態機 - 組合邏輯
always @(*) begin
    case(state)
        IDLE: begin
            next_state = INPUT;
        end
        
        INPUT: begin
            if(cnt == 149)
                next_state = CALC;
            else
                next_state = INPUT;
        end
        
        CALC: begin
            if(calc_cnt == 149)
                next_state = PEAK_SELECT;  // 計算完成後進行峰值選擇
            else
                next_state = CALC;
        end
        
        PEAK_SELECT: begin
            next_state = OUTPUT;
        end
        
        OUTPUT: begin
            next_state = INPUT;
        end
        
        default: next_state = IDLE;
    endcase
end

// 主要運算邏輯
always @(posedge clk or posedge rst) begin
    if(rst) begin
        cnt <= 0;
        calc_cnt <= 0;
        busy <= 0;
        valid <= 0;
        d <= 0;
        R_max <= 0;
        max_k <= 0;
        x_prev <= 0;  // 重置前一個樣本值
        y_prev <= 0;  // 重置前一個樣本值
        fine_offset <= 0;
        
        // 重置buffer
        for(i=0; i<150; i=i+1) begin
            x_buffer[i] <= 0;
            y_buffer[i] <= 0;
            R_values[i] <= 0;
        end
    end
    else begin
        case(state)
            IDLE: begin
                busy <= 0;
                valid <= 0;
            end
            
            INPUT: begin
                busy <= 0;
                valid <= 0;
                
                // 使用硬編碼數據，並進行濾波處理
                x_buffer[cnt] <= (x_hardcoded[cnt] + x_prev) >> 1;
                y_buffer[cnt] <= (y_hardcoded[cnt] + y_prev) >> 1;

                // 更新前一個樣本值
                x_prev <= x_hardcoded[cnt];
                y_prev <= y_hardcoded[cnt];
                
                if(cnt < 149)
                    cnt <= cnt + 1;
                else
                    cnt <= 0;
            end
            
            CALC: begin
                busy <= 1;
                valid <= 0;
                
                // 計算R(k)
                R_temp = 0;
                mul_temp = 0;
				case(calc_cnt)
					0: begin
						mul_temp = x_buffer[0] * y_buffer[0];
						R_temp = R_temp + mul_temp;
					end
					1: begin
						mul_temp = x_buffer[1] * y_buffer[1];
						R_temp = R_temp + mul_temp;
					end
					2: begin
						mul_temp = x_buffer[2] * y_buffer[2];
						R_temp = R_temp + mul_temp;
					end
					3: begin
						mul_temp = x_buffer[3] * y_buffer[3];
						R_temp = R_temp + mul_temp;
					end
					4: begin
						mul_temp = x_buffer[4] * y_buffer[4];
						R_temp = R_temp + mul_temp;
					end
					5: begin
						mul_temp = x_buffer[5] * y_buffer[5];
						R_temp = R_temp + mul_temp;
					end
					6: begin
						mul_temp = x_buffer[6] * y_buffer[6];
						R_temp = R_temp + mul_temp;
					end
					7: begin
						mul_temp = x_buffer[7] * y_buffer[7];
						R_temp = R_temp + mul_temp;
					end
					8: begin
						mul_temp = x_buffer[8] * y_buffer[8];
						R_temp = R_temp + mul_temp;
					end
					9: begin
						mul_temp = x_buffer[9] * y_buffer[9];
						R_temp = R_temp + mul_temp;
					end
					10: begin
						mul_temp = x_buffer[10] * y_buffer[10];
						R_temp = R_temp + mul_temp;
					end
					11: begin
						mul_temp = x_buffer[11] * y_buffer[11];
						R_temp = R_temp + mul_temp;
					end
					12: begin
						mul_temp = x_buffer[12] * y_buffer[12];
						R_temp = R_temp + mul_temp;
					end
					13: begin
						mul_temp = x_buffer[13] * y_buffer[13];
						R_temp = R_temp + mul_temp;
					end
					14: begin
						mul_temp = x_buffer[14] * y_buffer[14];
						R_temp = R_temp + mul_temp;
					end
					15: begin
						mul_temp = x_buffer[15] * y_buffer[15];
						R_temp = R_temp + mul_temp;
					end
					16: begin
						mul_temp = x_buffer[16] * y_buffer[16];
						R_temp = R_temp + mul_temp;
					end
					17: begin
						mul_temp = x_buffer[17] * y_buffer[17];
						R_temp = R_temp + mul_temp;
					end
					18: begin
						mul_temp = x_buffer[18] * y_buffer[18];
						R_temp = R_temp + mul_temp;
					end
					19: begin
						mul_temp = x_buffer[19] * y_buffer[19];
						R_temp = R_temp + mul_temp;
					end
					20: begin
						mul_temp = x_buffer[20] * y_buffer[20];
						R_temp = R_temp + mul_temp;
					end
					21: begin
						mul_temp = x_buffer[21] * y_buffer[21];
						R_temp = R_temp + mul_temp;
					end
					22: begin
						mul_temp = x_buffer[22] * y_buffer[22];
						R_temp = R_temp + mul_temp;
					end
					23: begin
						mul_temp = x_buffer[23] * y_buffer[23];
						R_temp = R_temp + mul_temp;
					end
					24: begin
						mul_temp = x_buffer[24] * y_buffer[24];
						R_temp = R_temp + mul_temp;
					end
					25: begin
						mul_temp = x_buffer[25] * y_buffer[25];
						R_temp = R_temp + mul_temp;
					end
					26: begin
						mul_temp = x_buffer[26] * y_buffer[26];
						R_temp = R_temp + mul_temp;
					end
					27: begin
						mul_temp = x_buffer[27] * y_buffer[27];
						R_temp = R_temp + mul_temp;
					end
					28: begin
						mul_temp = x_buffer[28] * y_buffer[28];
						R_temp = R_temp + mul_temp;
					end
					29: begin
						mul_temp = x_buffer[29] * y_buffer[29];
						R_temp = R_temp + mul_temp;
					end
					30: begin
						mul_temp = x_buffer[30] * y_buffer[30];
						R_temp = R_temp + mul_temp;
					end
					31: begin
						mul_temp = x_buffer[31] * y_buffer[31];
						R_temp = R_temp + mul_temp;
					end
					32: begin
						mul_temp = x_buffer[32] * y_buffer[32];
						R_temp = R_temp + mul_temp;
					end
					33: begin
						mul_temp = x_buffer[33] * y_buffer[33];
						R_temp = R_temp + mul_temp;
					end
					34: begin
						mul_temp = x_buffer[34] * y_buffer[34];
						R_temp = R_temp + mul_temp;
					end
					35: begin
						mul_temp = x_buffer[35] * y_buffer[35];
						R_temp = R_temp + mul_temp;
					end
					36: begin
						mul_temp = x_buffer[36] * y_buffer[36];
						R_temp = R_temp + mul_temp;
					end
					37: begin
						mul_temp = x_buffer[37] * y_buffer[37];
						R_temp = R_temp + mul_temp;
					end
					38: begin
						mul_temp = x_buffer[38] * y_buffer[38];
						R_temp = R_temp + mul_temp;
					end
					39: begin
						mul_temp = x_buffer[39] * y_buffer[39];
						R_temp = R_temp + mul_temp;
					end
					40: begin
						mul_temp = x_buffer[40] * y_buffer[40];
						R_temp = R_temp + mul_temp;
					end
					41: begin
						mul_temp = x_buffer[41] * y_buffer[41];
						R_temp = R_temp + mul_temp;
					end
					42: begin
						mul_temp = x_buffer[42] * y_buffer[42];
						R_temp = R_temp + mul_temp;
					end
					43: begin
						mul_temp = x_buffer[43] * y_buffer[43];
						R_temp = R_temp + mul_temp;
					end
					44: begin
						mul_temp = x_buffer[44] * y_buffer[44];
						R_temp = R_temp + mul_temp;
					end
					45: begin
						mul_temp = x_buffer[45] * y_buffer[45];
						R_temp = R_temp + mul_temp;
					end
					46: begin
						mul_temp = x_buffer[46] * y_buffer[46];
						R_temp = R_temp + mul_temp;
					end
					47: begin
						mul_temp = x_buffer[47] * y_buffer[47];
						R_temp = R_temp + mul_temp;
					end
					48: begin
						mul_temp = x_buffer[48] * y_buffer[48];
						R_temp = R_temp + mul_temp;
					end
					49: begin
						mul_temp = x_buffer[49] * y_buffer[49];
						R_temp = R_temp + mul_temp;
					end
					50: begin
						mul_temp = x_buffer[50] * y_buffer[50];
						R_temp = R_temp + mul_temp;
					end
					51: begin
						mul_temp = x_buffer[51] * y_buffer[51];
						R_temp = R_temp + mul_temp;
					end
					52: begin
						mul_temp = x_buffer[52] * y_buffer[52];
						R_temp = R_temp + mul_temp;
					end
					53: begin
						mul_temp = x_buffer[53] * y_buffer[53];
						R_temp = R_temp + mul_temp;
					end
					54: begin
						mul_temp = x_buffer[54] * y_buffer[54];
						R_temp = R_temp + mul_temp;
					end
					55: begin
						mul_temp = x_buffer[55] * y_buffer[55];
						R_temp = R_temp + mul_temp;
					end
					56: begin
						mul_temp = x_buffer[56] * y_buffer[56];
						R_temp = R_temp + mul_temp;
					end
					57: begin
						mul_temp = x_buffer[57] * y_buffer[57];
						R_temp = R_temp + mul_temp;
					end
					58: begin
						mul_temp = x_buffer[58] * y_buffer[58];
						R_temp = R_temp + mul_temp;
					end
					59: begin
						mul_temp = x_buffer[59] * y_buffer[59];
						R_temp = R_temp + mul_temp;
					end
					60: begin
						mul_temp = x_buffer[60] * y_buffer[60];
						R_temp = R_temp + mul_temp;
					end
					61: begin
						mul_temp = x_buffer[61] * y_buffer[61];
						R_temp = R_temp + mul_temp;
					end
					62: begin
						mul_temp = x_buffer[62] * y_buffer[62];
						R_temp = R_temp + mul_temp;
					end
					63: begin
						mul_temp = x_buffer[63] * y_buffer[63];
						R_temp = R_temp + mul_temp;
					end
					64: begin
						mul_temp = x_buffer[64] * y_buffer[64];
						R_temp = R_temp + mul_temp;
					end
					65: begin
						mul_temp = x_buffer[65] * y_buffer[65];
						R_temp = R_temp + mul_temp;
					end
					66: begin
						mul_temp = x_buffer[66] * y_buffer[66];
						R_temp = R_temp + mul_temp;
					end
					67: begin
						mul_temp = x_buffer[67] * y_buffer[67];
						R_temp = R_temp + mul_temp;
					end
					68: begin
						mul_temp = x_buffer[68] * y_buffer[68];
						R_temp = R_temp + mul_temp;
					end
					69: begin
						mul_temp = x_buffer[69] * y_buffer[69];
						R_temp = R_temp + mul_temp;
					end
					70: begin
						mul_temp = x_buffer[70] * y_buffer[70];
						R_temp = R_temp + mul_temp;
					end
					71: begin
						mul_temp = x_buffer[71] * y_buffer[71];
						R_temp = R_temp + mul_temp;
					end
					72: begin
						mul_temp = x_buffer[72] * y_buffer[72];
						R_temp = R_temp + mul_temp;
					end
					73: begin
						mul_temp = x_buffer[73] * y_buffer[73];
						R_temp = R_temp + mul_temp;
					end
					74: begin
						mul_temp = x_buffer[74] * y_buffer[74];
						R_temp = R_temp + mul_temp;
					end
					75: begin
						mul_temp = x_buffer[75] * y_buffer[75];
						R_temp = R_temp + mul_temp;
					end
					76: begin
						mul_temp = x_buffer[76] * y_buffer[76];
						R_temp = R_temp + mul_temp;
					end
					77: begin
						mul_temp = x_buffer[77] * y_buffer[77];
						R_temp = R_temp + mul_temp;
					end
					78: begin
						mul_temp = x_buffer[78] * y_buffer[78];
						R_temp = R_temp + mul_temp;
					end
					79: begin
						mul_temp = x_buffer[79] * y_buffer[79];
						R_temp = R_temp + mul_temp;
					end
					80: begin
						mul_temp = x_buffer[80] * y_buffer[80];
						R_temp = R_temp + mul_temp;
					end
					81: begin
						mul_temp = x_buffer[81] * y_buffer[81];
						R_temp = R_temp + mul_temp;
					end
					82: begin
						mul_temp = x_buffer[82] * y_buffer[82];
						R_temp = R_temp + mul_temp;
					end
					83: begin
						mul_temp = x_buffer[83] * y_buffer[83];
						R_temp = R_temp + mul_temp;
					end
					84: begin
						mul_temp = x_buffer[84] * y_buffer[84];
						R_temp = R_temp + mul_temp;
					end
					85: begin
						mul_temp = x_buffer[85] * y_buffer[85];
						R_temp = R_temp + mul_temp;
					end
					86: begin
						mul_temp = x_buffer[86] * y_buffer[86];
						R_temp = R_temp + mul_temp;
					end
					87: begin
						mul_temp = x_buffer[87] * y_buffer[87];
						R_temp = R_temp + mul_temp;
					end
					88: begin
						mul_temp = x_buffer[88] * y_buffer[88];
						R_temp = R_temp + mul_temp;
					end
					89: begin
						mul_temp = x_buffer[89] * y_buffer[89];
						R_temp = R_temp + mul_temp;
					end
					90: begin
						mul_temp = x_buffer[90] * y_buffer[90];
						R_temp = R_temp + mul_temp;
					end
					91: begin
						mul_temp = x_buffer[91] * y_buffer[91];
						R_temp = R_temp + mul_temp;
					end
					92: begin
						mul_temp = x_buffer[92] * y_buffer[92];
						R_temp = R_temp + mul_temp;
					end
					93: begin
						mul_temp = x_buffer[93] * y_buffer[93];
						R_temp = R_temp + mul_temp;
					end
					94: begin
						mul_temp = x_buffer[94] * y_buffer[94];
						R_temp = R_temp + mul_temp;
					end
					95: begin
						mul_temp = x_buffer[95] * y_buffer[95];
						R_temp = R_temp + mul_temp;
					end
					96: begin
						mul_temp = x_buffer[96] * y_buffer[96];
						R_temp = R_temp + mul_temp;
					end
					97: begin
						mul_temp = x_buffer[97] * y_buffer[97];
						R_temp = R_temp + mul_temp;
					end
					98: begin
						mul_temp = x_buffer[98] * y_buffer[98];
						R_temp = R_temp + mul_temp;
					end
					99: begin
						mul_temp = x_buffer[99] * y_buffer[99];
						R_temp = R_temp + mul_temp;
					end
					100: begin
						mul_temp = x_buffer[100] * y_buffer[100];
						R_temp = R_temp + mul_temp;
					end
					101: begin
						mul_temp = x_buffer[101] * y_buffer[101];
						R_temp = R_temp + mul_temp;
					end
					102: begin
						mul_temp = x_buffer[102] * y_buffer[102];
						R_temp = R_temp + mul_temp;
					end
					103: begin
						mul_temp = x_buffer[103] * y_buffer[103];
						R_temp = R_temp + mul_temp;
					end
					104: begin
						mul_temp = x_buffer[104] * y_buffer[104];
						R_temp = R_temp + mul_temp;
					end
					105: begin
						mul_temp = x_buffer[105] * y_buffer[105];
						R_temp = R_temp + mul_temp;
					end
					106: begin
						mul_temp = x_buffer[106] * y_buffer[106];
						R_temp = R_temp + mul_temp;
					end
					107: begin
						mul_temp = x_buffer[107] * y_buffer[107];
						R_temp = R_temp + mul_temp;
					end
					108: begin
						mul_temp = x_buffer[108] * y_buffer[108];
						R_temp = R_temp + mul_temp;
					end
					109: begin
						mul_temp = x_buffer[109] * y_buffer[109];
						R_temp = R_temp + mul_temp;
					end
					110: begin
						mul_temp = x_buffer[110] * y_buffer[110];
						R_temp = R_temp + mul_temp;
					end
					111: begin
						mul_temp = x_buffer[111] * y_buffer[111];
						R_temp = R_temp + mul_temp;
					end
					112: begin
						mul_temp = x_buffer[112] * y_buffer[112];
						R_temp = R_temp + mul_temp;
					end
					113: begin
						mul_temp = x_buffer[113] * y_buffer[113];
						R_temp = R_temp + mul_temp;
					end
					114: begin
						mul_temp = x_buffer[114] * y_buffer[114];
						R_temp = R_temp + mul_temp;
					end
					115: begin
						mul_temp = x_buffer[115] * y_buffer[115];
						R_temp = R_temp + mul_temp;
					end
					116: begin
						mul_temp = x_buffer[116] * y_buffer[116];
						R_temp = R_temp + mul_temp;
					end
					117: begin
						mul_temp = x_buffer[117] * y_buffer[117];
						R_temp = R_temp + mul_temp;
					end
					118: begin
						mul_temp = x_buffer[118] * y_buffer[118];
						R_temp = R_temp + mul_temp;
					end
					119: begin
						mul_temp = x_buffer[119] * y_buffer[119];
						R_temp = R_temp + mul_temp;
					end
					120: begin
						mul_temp = x_buffer[120] * y_buffer[120];
						R_temp = R_temp + mul_temp;
					end
					121: begin
						mul_temp = x_buffer[121] * y_buffer[121];
						R_temp = R_temp + mul_temp;
					end
					122: begin
						mul_temp = x_buffer[122] * y_buffer[122];
						R_temp = R_temp + mul_temp;
					end
					123: begin
						mul_temp = x_buffer[123] * y_buffer[123];
						R_temp = R_temp + mul_temp;
					end
					124: begin
						mul_temp = x_buffer[124] * y_buffer[124];
						R_temp = R_temp + mul_temp;
					end
					125: begin
						mul_temp = x_buffer[125] * y_buffer[125];
						R_temp = R_temp + mul_temp;
					end
					126: begin
						mul_temp = x_buffer[126] * y_buffer[126];
						R_temp = R_temp + mul_temp;
					end
					127: begin
						mul_temp = x_buffer[127] * y_buffer[127];
						R_temp = R_temp + mul_temp;
					end
					128: begin
						mul_temp = x_buffer[128] * y_buffer[128];
						R_temp = R_temp + mul_temp;
					end
					129: begin
						mul_temp = x_buffer[129] * y_buffer[129];
						R_temp = R_temp + mul_temp;
					end
					130: begin
						mul_temp = x_buffer[130] * y_buffer[130];
						R_temp = R_temp + mul_temp;
					end
					131: begin
						mul_temp = x_buffer[131] * y_buffer[131];
						R_temp = R_temp + mul_temp;
					end
					132: begin
						mul_temp = x_buffer[132] * y_buffer[132];
						R_temp = R_temp + mul_temp;
					end
					133: begin
						mul_temp = x_buffer[133] * y_buffer[133];
						R_temp = R_temp + mul_temp;
					end
					134: begin
						mul_temp = x_buffer[134] * y_buffer[134];
						R_temp = R_temp + mul_temp;
					end
					135: begin
						mul_temp = x_buffer[135] * y_buffer[135];
						R_temp = R_temp + mul_temp;
					end
					136: begin
						mul_temp = x_buffer[136] * y_buffer[136];
						R_temp = R_temp + mul_temp;
					end
					137: begin
						mul_temp = x_buffer[137] * y_buffer[137];
						R_temp = R_temp + mul_temp;
					end
					138: begin
						mul_temp = x_buffer[138] * y_buffer[138];
						R_temp = R_temp + mul_temp;
					end
					139: begin
						mul_temp = x_buffer[139] * y_buffer[139];
						R_temp = R_temp + mul_temp;
					end
					140: begin
						mul_temp = x_buffer[140] * y_buffer[140];
						R_temp = R_temp + mul_temp;
					end
					141: begin
						mul_temp = x_buffer[141] * y_buffer[141];
						R_temp = R_temp + mul_temp;
					end
					142: begin
						mul_temp = x_buffer[142] * y_buffer[142];
						R_temp = R_temp + mul_temp;
					end
					143: begin
						mul_temp = x_buffer[143] * y_buffer[143];
						R_temp = R_temp + mul_temp;
					end
					144: begin
						mul_temp = x_buffer[144] * y_buffer[144];
						R_temp = R_temp + mul_temp;
					end
					145: begin
						mul_temp = x_buffer[145] * y_buffer[145];
						R_temp = R_temp + mul_temp;
					end
					146: begin
						mul_temp = x_buffer[146] * y_buffer[146];
						R_temp = R_temp + mul_temp;
					end
					147: begin
						mul_temp = x_buffer[147] * y_buffer[147];
						R_temp = R_temp + mul_temp;
					end
					148: begin
						mul_temp = x_buffer[148] * y_buffer[148];
						R_temp = R_temp + mul_temp;
					end
					149: begin
						mul_temp = x_buffer[149] * y_buffer[149];
						R_temp = R_temp + mul_temp;
					end
					default: R_temp = 0;
				endcase
                
                // 更新最大R值
                if(R_temp > R_max) begin
                    R_max <= R_temp;
                    max_k <= calc_cnt;
                end
                
                if(calc_cnt < 149)
                    calc_cnt <= calc_cnt + 1;
                else
                    calc_cnt <= 0;
            end
            
            PEAK_SELECT: begin
                busy <= 1;
                valid <= 0;
                
                // 取出峰值點及其前後的R值
                if(max_k > 0 && max_k < 149) begin
                    R_before <= R_values[max_k-1];
                    R_peak <= R_values[max_k];
                    R_after <= R_values[max_k+1];
                    
                    // 二次插值計算
                    // delta = 0.5 * (R[k-1] - R[k+1]) / (2*R[k] - R[k-1] - R[k+1])
                    interp_num <= (R_values[max_k-1] - R_values[max_k+1]) * 128;  // 乘以256/2以保持精度
                    interp_den <= (2 * R_values[max_k] - R_values[max_k-1] - R_values[max_k+1]) * 2;
                    
                    if(interp_den != 0) begin
                        fine_offset <= interp_num / interp_den;  // 結果會是一個小數
                    end
                    else begin
                        fine_offset <= 0;
                    end
                end
            end
            
            OUTPUT: begin
                busy <= 0;
                valid <= 1;
                // 輸出經過插值校正的延遲值
                d <= max_k + (fine_offset >> 8);  // 將fine_offset轉換回實際的偏移量
                // 重置為下一次計算做準備
                R_max <= 0;
				R_values[0] <= 0;
				R_values[1] <= 0;
				R_values[2] <= 0;
				R_values[3] <= 0;
				R_values[4] <= 0;
				R_values[5] <= 0;
				R_values[6] <= 0;
				R_values[7] <= 0;
				R_values[8] <= 0;
				R_values[9] <= 0;
				R_values[10] <= 0;
				R_values[11] <= 0;
				R_values[12] <= 0;
				R_values[13] <= 0;
				R_values[14] <= 0;
				R_values[15] <= 0;
				R_values[16] <= 0;
				R_values[17] <= 0;
				R_values[18] <= 0;
				R_values[19] <= 0;
				R_values[20] <= 0;
				R_values[21] <= 0;
				R_values[22] <= 0;
				R_values[23] <= 0;
				R_values[24] <= 0;
				R_values[25] <= 0;
				R_values[26] <= 0;
				R_values[27] <= 0;
				R_values[28] <= 0;
				R_values[29] <= 0;
				R_values[30] <= 0;
				R_values[31] <= 0;
				R_values[32] <= 0;
				R_values[33] <= 0;
				R_values[34] <= 0;
				R_values[35] <= 0;
				R_values[36] <= 0;
				R_values[37] <= 0;
				R_values[38] <= 0;
				R_values[39] <= 0;
				R_values[40] <= 0;
				R_values[41] <= 0;
				R_values[42] <= 0;
				R_values[43] <= 0;
				R_values[44] <= 0;
				R_values[45] <= 0;
				R_values[46] <= 0;
				R_values[47] <= 0;
				R_values[48] <= 0;
				R_values[49] <= 0;
				R_values[50] <= 0;
				R_values[51] <= 0;
				R_values[52] <= 0;
				R_values[53] <= 0;
				R_values[54] <= 0;
				R_values[55] <= 0;
				R_values[56] <= 0;
				R_values[57] <= 0;
				R_values[58] <= 0;
				R_values[59] <= 0;
				R_values[60] <= 0;
				R_values[61] <= 0;
				R_values[62] <= 0;
				R_values[63] <= 0;
				R_values[64] <= 0;
				R_values[65] <= 0;
				R_values[66] <= 0;
				R_values[67] <= 0;
				R_values[68] <= 0;
				R_values[69] <= 0;
				R_values[70] <= 0;
				R_values[71] <= 0;
				R_values[72] <= 0;
				R_values[73] <= 0;
				R_values[74] <= 0;
				R_values[75] <= 0;
				R_values[76] <= 0;
				R_values[77] <= 0;
				R_values[78] <= 0;
				R_values[79] <= 0;
				R_values[80] <= 0;
				R_values[81] <= 0;
				R_values[82] <= 0;
				R_values[83] <= 0;
				R_values[84] <= 0;
				R_values[85] <= 0;
				R_values[86] <= 0;
				R_values[87] <= 0;
				R_values[88] <= 0;
				R_values[89] <= 0;
				R_values[90] <= 0;
				R_values[91] <= 0;
				R_values[92] <= 0;
				R_values[93] <= 0;
				R_values[94] <= 0;
				R_values[95] <= 0;
				R_values[96] <= 0;
				R_values[97] <= 0;
				R_values[98] <= 0;
				R_values[99] <= 0;
				R_values[100] <= 0;
				R_values[101] <= 0;
				R_values[102] <= 0;
				R_values[103] <= 0;
				R_values[104] <= 0;
				R_values[105] <= 0;
				R_values[106] <= 0;
				R_values[107] <= 0;
				R_values[108] <= 0;
				R_values[109] <= 0;
				R_values[110] <= 0;
				R_values[111] <= 0;
				R_values[112] <= 0;
				R_values[113] <= 0;
				R_values[114] <= 0;
				R_values[115] <= 0;
				R_values[116] <= 0;
				R_values[117] <= 0;
				R_values[118] <= 0;
				R_values[119] <= 0;
				R_values[120] <= 0;
				R_values[121] <= 0;
				R_values[122] <= 0;
				R_values[123] <= 0;
				R_values[124] <= 0;
				R_values[125] <= 0;
				R_values[126] <= 0;
				R_values[127] <= 0;
				R_values[128] <= 0;
				R_values[129] <= 0;
				R_values[130] <= 0;
				R_values[131] <= 0;
				R_values[132] <= 0;
				R_values[133] <= 0;
				R_values[134] <= 0;
				R_values[135] <= 0;
				R_values[136] <= 0;
				R_values[137] <= 0;
				R_values[138] <= 0;
				R_values[139] <= 0;
				R_values[140] <= 0;
				R_values[141] <= 0;
				R_values[142] <= 0;
				R_values[143] <= 0;
				R_values[144] <= 0;
				R_values[145] <= 0;
				R_values[146] <= 0;
				R_values[147] <= 0;
				R_values[148] <= 0;
				R_values[149] <= 0;					
            end
        endcase
    end
end
seg7_display display (
    .clk(clk),
    .rst_n(~rst), // 使用反向的 rst 信號作為七段顯示器的復位
    .d(d),        // 將 TDE 模組的輸出 d 連接到七段顯示器
    .seg7(seg7),  // 七段顯示器段位信號輸出
    .seg7_sel(seg7_sel) // 七段顯示器位選信號輸出
);
endmodule


module seg7_display(
    input clk,
    input rst_n,
    input [7:0] d,          // 顯示輸入，改為 d
    output reg [7:0] seg7,  // 七段顯示器段位輸出
    output reg [3:0] seg7_sel // 七段顯示器位選
);

    // 暫存器宣告
    reg [3:0] seg7_temp [0:3]; // 儲存數字
    reg [1:0] seg7_count;      // 控制當前顯示的位
    reg [29:0] count;          // 用於除頻
    wire d_clk;                // 分頻後的時鐘信號

    // 除頻
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            count <= 0;
        else if (count >= `CYCLE)
            count <= 0;
        else
            count <= count + 1;
    end
    assign d_clk = (count > (`CYCLE / 2)) ? 0 : 1;

    // 將輸入的 d 二進位轉換為十進位數字
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg7_temp[0] <= 0;
            seg7_temp[1] <= 0;
            seg7_temp[2] <= 0;
            seg7_temp[3] <= 0;
        end else begin
            seg7_temp[3] <= 0;                // d 的最高位固定為 0（因為 d 是 8 位）
            seg7_temp[2] <= (d % 1000) / 100; // 百位
            seg7_temp[1] <= (d % 100) / 10;   // 十位
            seg7_temp[0] <= d % 10;           // 個位
        end
    end

    // 控制當前顯示的位數
    always @(posedge d_clk or negedge rst_n) begin
        if (!rst_n)
            seg7_count <= 0;
        else
            seg7_count <= seg7_count + 1;
    end

    // 七段顯示器顯示邏輯
    always @(posedge d_clk or negedge rst_n) begin
        if (!rst_n) begin
			seg7_sel <= 0;
			seg7 <= 0;
        end else begin
            // 控制當前顯示的位選信號
            case (seg7_count)
                0: seg7_sel <= 4'b0001; // 顯示個位
                1: seg7_sel <= 4'b0010; // 顯示十位
                2: seg7_sel <= 4'b0100; // 顯示百位
                3: seg7_sel <= 4'b1000; // 顯示千位
            endcase

            // 將當前位的數字轉換為七段顯示器的段位信號
            case (seg7_temp[seg7_count])
                0: seg7 <= 8'b0011_1111;
                1: seg7 <= 8'b0000_0110;
                2: seg7 <= 8'b0101_1011;
                3: seg7 <= 8'b0100_1111;
                4: seg7 <= 8'b0110_0110;
                5: seg7 <= 8'b0110_1101;
                6: seg7 <= 8'b0111_1101;
                7: seg7 <= 8'b0000_0111;
                8: seg7 <= 8'b0111_1111;
                9: seg7 <= 8'b0110_1111;
                default: seg7 <= 8'b0000_0000; // 關閉顯示
            endcase
        end
    end

endmodule