`define CYCLE 50000 // 單一cycle 長度
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
            seg7_sel <= 4'b1111; // 禁用所有位
            seg7 <= 8'b0000_0000; // 清空顯示
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