`timescale 1ns/1ps

module TDE_testbench;

// 信號定義
reg clk;
reg rst;
reg [9:0] x;  // 傳出波訊號 10 位元
reg [9:0] y;  // 接收波訊號 10 位元
wire busy;
wire valid;
wire [7:0] d;  // 距離輸出，保持為 8 位元

// 模組實例化
TDE uut (
    .clk(clk),
    .rst(rst),
    .x(x),
    .y(y),
    .busy(busy),
    .valid(valid),
    .d(d)
);

// 時鐘生成
initial begin
    clk = 0;
    forever #5 clk = ~clk;  // 10ns 時鐘週期
end

// 測試數據定義
reg [9:0] x_test [0:149];  // 傳出波數據 10 位元
reg [9:0] y_test [0:149];  // 接收波數據 10 位元
integer i;
integer correct_count;  // 計數器，用來統計正確的次數

initial begin
    // 初始化
    rst = 1;
    x = 10'd0;
    y = 10'd0;
    correct_count = 0;  // 初始化計數器

    // 重置信號
    #20 rst = 0;

    // --- 第一次測試 ---
    // 生成 x 波形：中間有一個峰值
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 70 && i < 80)
            x_test[i] = $urandom_range(180, 220);  // 峰值
        else
            x_test[i] = $urandom_range(40, 60);   // 基準值
    end

    // 生成 y 波形：與 x 波形相似但延遲 30 個單位，並有衰減
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 0 && i < 100)
            y_test[i] = $urandom_range(40, 60);   // 前面的基準值
        else if(i >= 110)
            y_test[i] = $urandom_range(40, 60);   // 後面的基準值
        else if(i >= 100 && i < 110)
            y_test[i] = $urandom_range(150, 200);  // 峰值(有衰減)
        else
            y_test[i] = $urandom_range(40, 60);   // 基準值
    end

    // 輸入數據到模組（第一次測試 150 筆資料）
    for(i = 0; i < 150; i = i + 1) begin
        #10 x = x_test[i];
             y = y_test[i];
    end

    // 等待 valid 信號為 1
    wait(valid) begin
        #10;  // 等待 valid 穩定
        // 檢查輸出：如果 valid 且 d 等於 30，顯示 "correct"
        if (d == 30) begin
            correct_count = correct_count + 1;  // 正確的次數加一
        end else begin
            $display("Simulation Failed: No valid output");
        end
    end

    // --- 第二次測試 ---
    // 初始化
    rst = 1;
    x = 10'd0;
    y = 10'd0;

    // 重置信號
    #20 rst = 0;

    // 生成 x 波形：中間有一個峰值
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 70 && i < 80)
            x_test[i] = $urandom_range(180, 220);  // 峰值
        else
            x_test[i] = $urandom_range(40, 60);   // 基準值
    end

    // 生成 y 波形：與 x 波形相似但延遲 30 個單位，並有衰減
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 0 && i < 100)
            y_test[i] = $urandom_range(40, 60);   // 前面的基準值
        else if(i >= 110)
            y_test[i] = $urandom_range(40, 60);   // 後面的基準值
        else if(i >= 100 && i < 110)
            y_test[i] = $urandom_range(150, 200);  // 峰值(有衰減)
        else
            y_test[i] = $urandom_range(40, 60);   // 基準值
    end

    // 輸入數據到模組（第二次測試 150 筆資料）
    for(i = 0; i < 150; i = i + 1) begin
        #10 x = x_test[i];
             y = y_test[i];
    end

    // 等待 valid 信號為 1
    wait(valid) begin
        #10;  // 等待 valid 穩定
        // 檢查輸出：如果 valid 且 d 等於 30，顯示 "correct"
        if (d == 30) begin
            correct_count = correct_count + 1;  // 正確的次數加一
        end else begin
            $display("Simulation Failed: No valid output");
        end
    end

    // --- 第三次測試 ---
    // 初始化
    rst = 1;
    x = 10'd0;
    y = 10'd0;

    // 重置信號
    #20 rst = 0;

    // 生成 x 波形：中間有一個峰值
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 70 && i < 80)
            x_test[i] = $urandom_range(180, 220);  // 峰值
        else
            x_test[i] = $urandom_range(40, 80);   // 基準值
    end

    // 生成 y 波形：與 x 波形相似但延遲 30 個單位，並有衰減
    for(i = 0; i < 150; i = i + 1) begin
        if(i >= 0 && i < 100)
            y_test[i] = $urandom_range(40, 80);   // 前面的基準值
        else if(i >= 110)
            y_test[i] = $urandom_range(40, 80);   // 後面的基準值
        else if(i >= 100 && i < 110)
            y_test[i] = $urandom_range(150, 200);  // 峰值(有衰減)
        else
            y_test[i] = $urandom_range(40, 80);   // 基準值
    end

    // 輸入數據到模組（第三次測試 150 筆資料）
    for(i = 0; i < 150; i = i + 1) begin
        #10 x = x_test[i];
             y = y_test[i];
    end

    // 等待 valid 信號為 1
    wait(valid) begin
        #10;  // 等待 valid 穩定
        // 檢查輸出：如果 valid 且 d 等於 30，顯示 "correct"
        if (d == 30) begin
            correct_count = correct_count + 1;  // 正確的次數加一
        end else begin
            $display("Simulation Failed: No valid output");
        end
    end

    // 打印計數器的最終結果
    $display("Total correct matches: %d ", correct_count);

    // 停止模擬
    $stop;
end

endmodule
