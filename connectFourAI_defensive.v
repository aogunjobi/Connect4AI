// Defensive AI Module for Connect Four Game
module connectFourAI_defensive(
    input [83:0] gameState,
    output [3:0] aiMove,
    output [4:0] maxConnectOut,
    input clk, reset
);
    wire [13:0] rowArray[5:0];

    // Split gameState into rows
    assign {rowArray[5], rowArray[4], rowArray[3], rowArray[2], rowArray[1], rowArray[0]} = gameState;
    
    wire [4:0] maxConnect[6:0];

    // Instantiate seven maxConnectFinder_defensive modules for each column
    genvar i;
    generate
        for (i = 0; i < 7; i = i + 1) begin: maxConnectFinders
            maxConnectFinder_defensive maxFinder (
                .clk(clk),
                .row0(rowArray[0]),
                .row1(rowArray[1]),
                .row2(rowArray[2]),
                .row3(rowArray[3]),
                .row4(rowArray[4]),
                .row5(rowArray[5]),
                .moveColumn(i[3:0]),
                .maxConnect(maxConnect[i])
            );
        end
    endgenerate

    wire [4:0] foundMax;

    // Find maximum heuristic value
    findMaximum_defensive findMax (
        .in0(maxConnect[0]),
        .in1(maxConnect[1]),
        .in2(maxConnect[2]),
        .in3(maxConnect[3]),
        .in4(maxConnect[4]),
        .in5(maxConnect[5]),
        .in6(maxConnect[6]),
        .out(foundMax)
    );

    // Determine AI move based on maximum heuristic value
    assign aiMove = (foundMax == maxConnect[3]) ? 3 :
                    (foundMax == maxConnect[2]) ? 2 :
                    (foundMax == maxConnect[4]) ? 4 :
                    (foundMax == maxConnect[1]) ? 1 :
                    (foundMax == maxConnect[5]) ? 5 :
                    (foundMax == maxConnect[0]) ? 0 : 6;
    assign maxConnectOut = foundMax;
endmodule

// Compute heuristic value for a given game state and move column
module maxConnectFinder_defensive(
    input [13:0] row0, row1, row2, row3, row4, row5,
    input [3:0] moveColumn,
    input clk,
    output [4:0] maxConnect
);
    wire [13:0] rowArray[5:0] = {row5, row4, row3, row2, row1, row0};
    wire [3:0] moveRow = row0[moveColumn*2 +: 2] + row1[moveColumn*2 +: 2] +
                         row2[moveColumn*2 +: 2] + row3[moveColumn*2 +: 2] +
                         row4[moveColumn*2 +: 2] + row5[moveColumn*2 +: 2];
    
    reg [3:0] upLeft, upRight, left, right, bot, botRight, botLeft;
    reg [3:0] upLeftSelf, upRightSelf, leftSelf, rightSelf, botSelf, botRightSelf, botLeftSelf;

    wire crossCheckSelf = ((upLeftSelf + botRightSelf) >= 4'd4 || 
                           (upRightSelf + botLeftSelf) >= 4'd4 ||
                           (leftSelf + rightSelf) >= 4'd4);
    wire selfWinWire = (botSelf == 4'd10 || botLeftSelf == 4'd10 || botRightSelf == 4'd10 ||
                        leftSelf == 4'd10 || rightSelf == 4'd10 || upLeftSelf == 4'd10 ||
                        upRightSelf == 4'd10 || crossCheckSelf);
    wire [4:0] crossCheck = ((upLeft + botRight) >= 4'd4 || 
                             (upRight + botLeft) >= 4'd4 || 
                             (left + right) >= 4'd4) ? 5'd10 : 5'd0;
    wire [4:0] tempSum = bot + botLeft + botRight + left + right + upLeft + upRight + crossCheck;

    assign maxConnect = (moveRow < 4'd6) ? (selfWinWire ? 5'd30 : tempSum) : 5'd0;

    always @(*) begin
        // Check for opponent moves
        bot = (moveRow == 0) ? 4'd0 : (rowArray[moveRow-1][moveColumn*2+1] == 1'b1) ? 
              ((moveRow-1 != 0 && rowArray[moveRow-2][moveColumn*2+1] == 1'b1) ? 
              ((moveRow-2 != 0 && rowArray[moveRow-3][moveColumn*2+1] == 1'b1) ? 4'd15 : 4'd5) : 4'd1) : 4'd0;

        botLeft = (moveRow == 0 || moveColumn == 0) ? 4'd0 : (rowArray[moveRow-1][(moveColumn-1)*2+1] == 1'b1) ? 
                  ((moveRow-1 != 0 && moveColumn-1 != 0 && rowArray[moveRow-2][(moveColumn-2)*2+1] == 1'b1) ? 
                  ((moveRow-2 != 0 && moveColumn-2 != 0 && rowArray[moveRow-3][(moveColumn-3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        botRight = (moveRow == 0 || moveColumn == 6) ? 4'd0 : (rowArray[moveRow-1][(moveColumn+1)*2+1] == 1'b1) ? 
                   ((moveRow-1 != 0 && moveColumn+1 != 6 && rowArray[moveRow-2][(moveColumn+2)*2+1] == 1'b1) ? 
                   ((moveRow-2 != 0 && moveColumn+2 != 6 && rowArray[moveRow-3][(moveColumn+3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        left = (moveColumn == 0) ? 4'd0 : (rowArray[moveRow][(moveColumn-1)*2+1] == 1'b1) ? 
               ((moveColumn-1 != 0 && rowArray[moveRow][(moveColumn-2)*2+1] == 1'b1) ? 
               ((moveColumn-2 != 0 && rowArray[moveRow][(moveColumn-3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        right = (moveColumn == 6) ? 4'd0 : (rowArray[moveRow][(moveColumn+1)*2+1] == 1'b1) ? 
                ((moveColumn+1 != 6 && rowArray[moveRow][(moveColumn+2)*2+1] == 1'b1) ? 
                ((moveColumn+2 != 6 && rowArray[moveRow][(moveColumn+3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        upLeft = (moveColumn == 0 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn-1)*2+1] == 1'b1) ? 
                 ((moveColumn-1 != 0 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn-2)*2+1] == 1'b1) ? 
                 ((moveColumn-2 != 0 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn-3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        upRight = (moveColumn == 6 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn+1)*2+1] == 1'b1) ? 
                  ((moveColumn+1 != 6 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn+2)*2+1] == 1'b1) ? 
                  ((moveColumn+2 != 6 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn+3)*2+1] == 1'b1) ? 4'd10 : 4'd5) : 4'd1) : 4'd0;

        // Check for AI win
        botSelf = (moveRow == 0) ? 4'd0 : (rowArray[moveRow-1][moveColumn*2] == 1'b1) ? 
                  ((moveRow-1 != 0 && rowArray[moveRow-2][moveColumn*2] == 1'b1) ? 
                  ((moveRow-2 != 0 && rowArray[moveRow-3][moveColumn*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        botLeftSelf = (moveRow == 0 || moveColumn == 0) ? 4'd0 : (rowArray[moveRow-1][(moveColumn-1)*2] == 1'b1) ? 
                      ((moveRow-1 != 0 && moveColumn-1 != 0 && rowArray[moveRow-2][(moveColumn-2)*2] == 1'b1) ? 
                      ((moveRow-2 != 0 && moveColumn-2 != 0 && rowArray[moveRow-3][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        botRightSelf = (moveRow == 0 || moveColumn == 6) ? 4'd0 : (rowArray[moveRow-1][(moveColumn+1)*2] == 1'b1) ? 
                       ((moveRow-1 != 0 && moveColumn+1 != 6 && rowArray[moveRow-2][(moveColumn+2)*2] == 1'b1) ? 
                       ((moveRow-2 != 0 && moveColumn+2 != 6 && rowArray[moveRow-3][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        leftSelf = (moveColumn == 0) ? 4'd0 : (rowArray[moveRow][(moveColumn-1)*2] == 1'b1) ? 
                   ((moveColumn-1 != 0 && rowArray[moveRow][(moveColumn-2)*2] == 1'b1) ? 
                   ((moveColumn-2 != 0 && rowArray[moveRow][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        rightSelf = (moveColumn == 6) ? 4'd0 : (rowArray[moveRow][(moveColumn+1)*2] == 1'b1) ? 
                    ((moveColumn+1 != 6 && rowArray[moveRow][(moveColumn+2)*2] == 1'b1) ? 
                    ((moveColumn+2 != 6 && rowArray[moveRow][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        upLeftSelf = (moveColumn == 0 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn-1)*2] == 1'b1) ? 
                     ((moveColumn-1 != 0 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn-2)*2] == 1'b1) ? 
                     ((moveColumn-2 != 0 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        upRightSelf = (moveColumn == 6 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn+1)*2] == 1'b1) ? 
                      ((moveColumn+1 != 6 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn+2)*2] == 1'b1) ? 
                      ((moveColumn+2 != 6 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;
    end
endmodule

// Find maximum value among seven inputs
module findMaximum_defensive(
    input [4:0] in0, in1, in2, in3, in4, in5, in6,
    output [4:0] out
);
    wire [4:0] comp1 = (in0 > in1) ? in0 : in1;
    wire [4:0] comp2 = (in2 > in3) ? in2 : in3;
    wire [4:0] comp3 = (in4 > in5) ? in4 : in5;
    wire [4:0] comp4 = (comp1 > comp2) ? comp1 : comp2;
    wire [4:0] comp5 = (comp4 > comp3) ? comp4 : comp3;
    assign out = (comp5 > in6) ? comp5 : in6;
endmodule
