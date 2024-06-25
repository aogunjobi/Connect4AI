// Top-Level AI Module for Connect Four Game
// gamestate bits: 00 = empty, 01 = AI move, 10 = user move
module ConnectFourAI_TopModule(
    input [83:0] gameState,
    output [3:0] aiMove,
    output [4:0] maxConnectOut,
    input clk, reset
);
    wire [3:0] move[7:0];
    wire [4:0] foundMax[7:0];
    wire [83:0] nextState[6:0];
    wire disable_column[6:0];
    wire [3:0] trapMove[7:0];

    // Generate new game states for AI moves in each column
    genStateFromColumn genMove[6:0] (
        .column(4'd0 + $unsigned($clog2(genMove))),
        .gameState(gameState),
        .player(2'b01),
        .nextState(nextState),
        .disable_column(disable_column)
    );

    // Compute heuristic values for each new game state
    moveDeterminer findMove[7:0] (
        .gameState({gameState, nextState}),
        .clk(clk),
        .disable_column({1'b0, disable_column}),
        .player(2'b10),
        .foundMax(foundMax),
        .move(move)
    );

    // Find the minimum heuristic value
    wire [4:0] min;
    findMinimum minFinder(
        .in0(foundMax[0]), .in1(foundMax[1]), .in2(foundMax[2]),
        .in3(foundMax[3]), .in4(foundMax[4]), .in5(foundMax[5]),
        .in6(foundMax[6]), .out(min)
    );

    // Trap determination logic
    trapDeterminer trap[6:0] (
        .gameState(nextState),
        .player(2'b01),
        .clk(clk),
        .disable_column(disable_column),
        .move(trapMove)
    );

    // AI move selection logic
    wire [3:0] trapMoveFlag = (trapMove[3] != 4'd7) ? 4'd3 :
                              (trapMove[2] != 4'd7) ? 4'd2 :
                              (trapMove[4] != 4'd7) ? 4'd4 :
                              (trapMove[5] != 4'd7) ? 4'd5 :
                              (trapMove[1] != 4'd7) ? 4'd1 :
                              (trapMove[6] != 4'd7) ? 4'd6 : 4'd7;
    wire [3:0] minMove = (min == foundMax[3]) ? 4'd3 :
                         (min == foundMax[2]) ? 4'd2 :
                         (min == foundMax[4]) ? 4'd4 :
                         (min == foundMax[1]) ? 4'd1 :
                         (min == foundMax[5]) ? 4'd5 :
                         (min == foundMax[0]) ? 4'd0 : 4'd6;
    wire allFull = &{foundMax[0], foundMax[1], foundMax[2], foundMax[3], foundMax[4], foundMax[5], foundMax[6]};

    assign aiMove = (foundMax[7] >= 15 || allFull) ? move[7] :
                    (trapMoveFlag != 4'd7 && foundMax[trapMoveFlag] < 5'd15) ? trapMoveFlag : minMove;
    assign maxConnectOut = (foundMax[7] >= 5'd15 || allFull) ? foundMax[7] : min;
endmodule

// Module to compute heuristic max value and corresponding move
module moveDeterminer(
    input [83:0] gameState,
    input [1:0] player,
    input clk, disable_column,
    output [4:0] foundMax,
    output [3:0] move
);
    wire [13:0] row[5:0];
    assign {row[5], row[4], row[3], row[2], row[1], row[0]} = gameState;
    wire [4:0] maxConnect[6:0];

    maxConnectFinder col[6:0] (
        .clk(clk),
        .player(player),
        .row0(row[0]), .row1(row[1]), .row2(row[2]),
        .row3(row[3]), .row4(row[4]), .row5(row[5]),
        .moveColumn(4'd0 + $unsigned($clog2(col))),
        .maxConnect(maxConnect)
    );

    wire [4:0] foundMax_temp;
    findMaximum findmax(
        .in0(maxConnect[0]), .in1(maxConnect[1]), .in2(maxConnect[2]),
        .in3(maxConnect[3]), .in4(maxConnect[4]), .in5(maxConnect[5]),
        .in6(maxConnect[6]), .out(foundMax_temp)
    );

    assign foundMax = (disable_column) ? (player == 2'b10 ? 5'd31 : 5'd0) : foundMax_temp;
    assign move = (foundMax == maxConnect[3]) ? 4'd3 :
                  (foundMax == maxConnect[2]) ? 4'd2 :
                  (foundMax == maxConnect[4]) ? 4'd4 :
                  (foundMax == maxConnect[1]) ? 4'd1 :
                  (foundMax == maxConnect[5]) ? 4'd5 :
                  (foundMax == maxConnect[0]) ? 4'd0 : 4'd6;
endmodule

// Module to determine trap moves
module trapDeterminer(
    input [83:0] gameState,
    input [1:0] player,
    input clk, disable_column,
    output [3:0] move
);
    wire [13:0] row[5:0];
    assign {row[5], row[4], row[3], row[2], row[1], row[0]} = gameState;
    wire selfWin[6:0];

    winFinder win[6:0] (
        .clk(clk),
        .row0(row[0]), .row1(row[1]), .row2(row[2]),
        .row3(row[3]), .row4(row[4]), .row5(row[5]),
        .moveColumn(4'd0 + $unsigned($clog2(win))),
        .selfWin(selfWin)
    );

    wire [3:0] userMove = (disable_column) ? 4'd7 :
                          (selfWin[3]) ? 4'd3 :
                          (selfWin[2]) ? 4'd2 :
                          (selfWin[4]) ? 4'd4 :
                          (selfWin[1]) ? 4'd1 :
                          (selfWin[5]) ? 4'd5 :
                          (selfWin[0]) ? 4'd0 :
                          (selfWin[6]) ? 4'd6 : 4'd7;

    wire [83:0] trapState;
    wire tempDisable;
    genStateFromColumn genTrapMove(
        .column(userMove),
        .gameState(gameState),
        .player(2'b10),
        .nextState(trapState),
        .disable_column(tempDisable)
    );

    wire selfWinN[6:0];
    winFinder winN[6:0] (
        .clk(clk),
        .row0(trapState[13:0]), .row1(trapState[27:14]), .row2(trapState[41:28]),
        .row3(trapState[55:42]), .row4(trapState[69:56]), .row5(trapState[83:70]),
        .moveColumn(4'd0 + $unsigned($clog2(winN))),
        .selfWin(selfWinN)
    );

    assign move = (tempDisable) ? 4'd7 :
                  (|selfWinN) ? userMove : 4'd7;
endmodule

// Module to find maximum value from inputs
module findMaximum(
    input [4:0] in0, in1, in2, in3, in4, in5, in6,
    output [4:0] out
);
    wire [4:0] comp1 = (in0 == 5'd31) ? in1 : ((in1 == 5'd31) ? in0 : ((in0 > in1) ? in0 : in1));
    wire [4:0] comp2 = (in2 == 5'd31) ? in3 : ((in3 == 5'd31) ? in2 : ((in2 > in3) ? in2 : in3));
    wire [4:0] comp3 = (in4 == 5'd31) ? in5 : ((in5 == 5'd31) ? in4 : ((in4 > in5) ? in4 : in5));
    wire [4:0] comp4 = (comp1 == 5'd31) ? comp2 : ((comp2 == 5'd31) ? comp1 : ((comp1 > comp2) ? comp1 : comp2));
    wire [4:0] comp5 = (comp4 == 5'd31) ? comp3 : ((comp3 == 5'd31) ? comp4 : ((comp4 > comp3) ? comp4 : comp3));
    assign out = (comp5 == 5'd31) ? in6 : ((in6 == 5'd31) ? comp5 : ((comp5 > in6) ? comp5 : in6));
endmodule

// Module to find minimum value from inputs
module findMinimum(
    input [4:0] in0, in1, in2, in3, in4, in5, in6,
    output [4:0] out
);
    wire [4:0] comp1 = (in0 < in1) ? in0 : in1;
    wire [4:0] comp2 = (in2 < in3) ? in2 : in3;
    wire [4:0] comp3 = (in4 < in5) ? in4 : in5;
    wire [4:0] comp4 = (comp1 < comp2) ? comp1 : comp2;
    wire [4:0] comp5 = (comp4 < comp3) ? comp4 : comp3;
    assign out = (comp5 < in6) ? comp5 : in6;
endmodule

// Generate next game state from current game state and a new move
module genStateFromColumn(
    input [3:0] column,
    input [83:0] gameState,
    input [1:0] player,
    output [83:0] nextState,
    output disable_column
);
    wire [13:0] row[5:0];
    assign {row[5], row[4], row[3], row[2], row[1], row[0]} = gameState;
    
    wire [3:0] moveRow = row[0][column*2 +: 2] + row[1][column*2 +: 2] +
                         row[2][column*2 +: 2] + row[3][column*2 +: 2] +
                         row[4][column*2 +: 2] + row[5][column*2 +: 2];
    assign disable_column = (moveRow >= 4'd6) ? 1'b1 : 1'b0;
    
    wire [13:0] newRow = row[moveRow] | (14'd1 << ((column*2) + player - 4'd1));
    assign nextState = {
        (moveRow == 5) ? newRow : row[5],
        (moveRow == 4) ? newRow : row[4],
        (moveRow == 3) ? newRow : row[3],
        (moveRow == 2) ? newRow : row[2],
        (moveRow == 1) ? newRow : row[1],
        (moveRow == 0) ? newRow : row[0]
    };
endmodule

// Module to detect potential winning move
module winFinder(
    input [13:0] row0, row1, row2, row3, row4, row5,
    input [3:0] moveColumn,
    input clk,
    output selfWin
);
    wire [13:0] rowArray[5:0] = {row5, row4, row3, row2, row1, row0};
    wire [3:0] moveRow = row0[moveColumn*2 +: 2] + row1[moveColumn*2 +: 2] +
                         row2[moveColumn*2 +: 2] + row3[moveColumn*2 +: 2] +
                         row4[moveColumn*2 +: 2] + row5[moveColumn*2 +: 2];
    reg [3:0] upLeftSelf, upRightSelf, leftSelf, rightSelf, botSelf, botRightSelf, botLeftSelf;
    
    // Check for three-in-a-rows next to an empty slot for AI tokens
    wire crossCheckSelf = ((upLeftSelf + botRightSelf) >= 4'd4 || (upRightSelf + botLeftSelf) >= 4'd4 ||
                          (leftSelf + rightSelf) >= 4'd4);
    assign selfWin = (botSelf == 4'd10 || botLeftSelf == 4'd10 || botRightSelf == 4'd10 ||
                      leftSelf == 4'd10 || rightSelf == 4'd10 || upLeftSelf == 4'd10 ||
                      upRightSelf == 4'd10 || crossCheckSelf);

    always @(*) begin
        // Bottom check
        botSelf = (moveRow == 0) ? 4'd0 : (rowArray[moveRow-1][moveColumn*2] == 1'b1) ? 
                  ((moveRow-1 != 0 && rowArray[moveRow-2][moveColumn*2] == 1'b1) ? 
                  ((moveRow-2 != 0 && rowArray[moveRow-3][moveColumn*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Bottom left check
        botLeftSelf = (moveRow == 0 || moveColumn == 0) ? 4'd0 : (rowArray[moveRow-1][(moveColumn-1)*2] == 1'b1) ? 
                      ((moveRow-1 != 0 && moveColumn-1 != 0 && rowArray[moveRow-2][(moveColumn-2)*2] == 1'b1) ? 
                      ((moveRow-2 != 0 && moveColumn-2 != 0 && rowArray[moveRow-3][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Bottom right check
        botRightSelf = (moveRow == 0 || moveColumn == 6) ? 4'd0 : (rowArray[moveRow-1][(moveColumn+1)*2] == 1'b1) ? 
                       ((moveRow-1 != 0 && moveColumn+1 != 6 && rowArray[moveRow-2][(moveColumn+2)*2] == 1'b1) ? 
                       ((moveRow-2 != 0 && moveColumn+2 != 6 && rowArray[moveRow-3][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Left check
        leftSelf = (moveColumn == 0) ? 4'd0 : (rowArray[moveRow][(moveColumn-1)*2] == 1'b1) ? 
                   ((moveColumn-1 != 0 && rowArray[moveRow][(moveColumn-2)*2] == 1'b1) ? 
                   ((moveColumn-2 != 0 && rowArray[moveRow][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Right check
        rightSelf = (moveColumn == 6) ? 4'd0 : (rowArray[moveRow][(moveColumn+1)*2] == 1'b1) ? 
                    ((moveColumn+1 != 6 && rowArray[moveRow][(moveColumn+2)*2] == 1'b1) ? 
                    ((moveColumn+2 != 6 && rowArray[moveRow][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Upper left check
        upLeftSelf = (moveColumn == 0 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn-1)*2] == 1'b1) ? 
                     ((moveColumn-1 != 0 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn-2)*2] == 1'b1) ? 
                     ((moveColumn-2 != 0 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn-3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;

        // Upper right check
        upRightSelf = (moveColumn == 6 || moveRow == 5) ? 4'd0 : (rowArray[moveRow+1][(moveColumn+1)*2] == 1'b1) ? 
                      ((moveColumn+1 != 6 && moveRow+1 != 5 && rowArray[moveRow+2][(moveColumn+2)*2] == 1'b1) ? 
                      ((moveColumn+2 != 6 && moveRow+2 != 5 && rowArray[moveRow+3][(moveColumn+3)*2] == 1'b1) ? 4'd10 : 4'd3) : 4'd1) : 4'd0;
    end
endmodule

// Module to compute heuristic value for a given game state and move column
module maxConnectFinder(
    input [13:0] row0, row1, row2, row3, row4, row5,
    input [3:0] moveColumn,
    input [1:0] player,
    input clk,
    output [4:0] maxConnect
);
    wire [13:0] rowArray[5:0] = {row5, row4, row3, row2, row1, row0};
    wire [3:0] moveRow = row0[moveColumn*2 +: 2] + row1[moveColumn*2 +: 2] +
                         row2[moveColumn*2 +: 2] + row3[moveColumn*2 +: 2] +
                         row4[moveColumn*2 +: 2] + row5[moveColumn*2 +: 2];
    reg [3:0] upLeft, upRight, left, right, bot, botRight, botLeft;
    reg [3:0] upLeftSelf, upRightSelf, leftSelf, rightSelf, botSelf, botRightSelf, botLeftSelf;
    
    wire crossCheckSelf = ((upLeftSelf + botRightSelf) >= 4'd4 || (upRightSelf + botLeftSelf) >= 4'd4 ||
                          (leftSelf + rightSelf) >= 4'd4);
    wire selfWinWire = (botSelf == 4'd10 || botLeftSelf == 4'd10 || botRightSelf == 4'd10 ||
                        leftSelf == 4'd10 || rightSelf == 4'd10 || upLeftSelf == 4'd10 ||
                        upRightSelf == 4'd10 || crossCheckSelf);
    wire [4:0] crossCheck = ((upLeft + botRight) >= 4'd6 || (upRight + botLeft) >= 4'd6 ||
                            (left + right) >= 4'd6) ? 5'd10 : 
                            ((upLeft + botRight == 4'd2 || upRight + botLeft == 4'd2 || 
                            left + right == 4'd2) ? 5'd3 : 5'd0);
    wire [4:0] tempSum = bot + botLeft + botRight + left + right + upLeft + upRight + crossCheck;
    assign maxConnect = (moveRow < 4'd6) ? ((selfWinWire && tempSum < 4'd15) ? 
                    (player == 2'b01 ? 5'd30 : 5'd14) : tempSum) : (player == 2'b01 ? 5'd0 : 5'd31);

    always @(*) begin
        // Opponent moves check
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

        // Self win check
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
