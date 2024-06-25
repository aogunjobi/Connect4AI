#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "address_map_arm_brl4.h"

// Define base addresses and spans
#define FPGA_AXI_BASE 0xC0000000
#define FPGA_AXI_SPAN 0x04001000

// Board Dimensions
#define NUM_ROWS 6
#define NUM_COLS 7

// Helper Functions
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

// configurable global variable to control hps-->fpga clock period
#define CLOCK_DELAY 1000000

#define FPGA_RESET_BASE 0x04000010

// initialize PIO port pointers
volatile int *reset_ptr = NULL;

// graphics primitives
void VGA_text(int, int, char *);
void VGA_text_clear();
void VGA_box(int, int, int, int, short);
void VGA_rect(int, int, int, int, short);
void VGA_line(int, int, int, int, short);
void VGA_Vline(int, int, int, short);
void VGA_Hline(int, int, int, short);
void VGA_disc(int, int, int, short);
void VGA_circle(int, int, int, int);

// 16-bit primary colors
#define red 0x00e0
#define dark_red 0x0015
#define green 0x07E0
#define dark_green 0x03E0
#define blue 0x001F
#define dark_blue 0x0099
#define yellow 0xFFE0
#define cyan 0x07FF
#define magenta 0xF81F
#define black 0x0000
#define gray 0x8410
#define white 0xFFFF

int colors[] = {red, dark_red, green, dark_green, blue, dark_blue,
                yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x, y, color) do { \
    char *pixel_ptr; \
    pixel_ptr = ((char *)vga_pixel_ptr + ((y) * 640 + (x))); \
    *(char *)pixel_ptr = (color); \
} while (0)

// the light weight bus base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int *vga_pixel_ptr = NULL;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int *vga_char_ptr = NULL;
void *vga_char_virtual_base;

// SRAM HPS-FPGA Interface Buffer
volatile unsigned int *sram_ptr = NULL;
void *sram_virtual_base;

// /dev/mem file id
int fd;

// measure time
struct timeval t1, t2;
double elapsedTime;

// helper function prototypes
char *decimalToBinary(int, int);
int binaryLongToInt(long);
int toSevenTwentyHex(char *);
void placeDisc(int, int, int);
void pullReset();
void updateGameState(int *, int, int, int);
int checkGameOver(int *, int *, int);

// initialization of reset loop variable
int resetStall = 0;

// initialize shared mutex variables and flags for resetting + pausing VGA from different threads
pthread_mutex_t reset_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t pause_lock = PTHREAD_MUTEX_INITIALIZER;
int resetVGA = 0;
int pauseVGA = 0;

// function to convert decimal numbers into 7.20 format
int toSevenTwentyHex(char *value) {
    char str[20];
    char s[2] = ".";
    char *token;
    strcpy(str, value);
    token = strtok(str, s);
    char *result1;
    result1 = decimalToBinary(atoi(token), 7);
    token = strtok(NULL, s);

    int length = strlen(token);
    float op1 = atoi(token) / (pow(10, length));
    op1 = op1 * 1048576;
    char *result2;
    result2 = decimalToBinary((int)op1, 20);
    strcat(result1, result2);

    long ret;
    char *placeholder;
    ret = strtol(result1, &placeholder, 2);
    return (int)ret;
}

// function to convert a decimal number to a binary number
char *decimalToBinary(int value, int bitLength) {
    int c, d, count;
    char *pointer;
    count = 0;
    pointer = (char *)malloc(bitLength + 1);
    if (pointer == NULL)
        exit(EXIT_FAILURE);

    for (c = bitLength - 1; c >= 0; c--) {
        d = value >> c;
        if (d & 1)
            *(pointer + count) = 1 + '0';
        else
            *(pointer + count) = 0 + '0';
        count++;
    }
    *(pointer + count) = '\0';
    return pointer;
}

// function to convert a binary number to an integer
int binaryLongToInt(long n) {
    int dec = 0, i = 0, rem;
    while (n != 0) {
        rem = n % 10;
        n /= 10;
        dec += rem * pow(2, i);
        ++i;
    }
    return dec;
}

void placeDisc(int x, int y, int player) {
    int box_width = 640 / NUM_COLS;
    int box_height = 480 / NUM_ROWS;
    if (x >= NUM_COLS || y >= NUM_ROWS || x < 0 || y < 0) {
        printf("ERROR: Invalid placement\n");
        return;
    }
    y = (NUM_ROWS - 1) - y;
    if (player == 1) {
        VGA_disc(x * box_width + (box_width / 2), y * box_height + (box_height / 2), MIN(box_width, box_height) * 0.40, red);
    } else if (player == 2) {
        VGA_disc(x * box_width + (box_width / 2), y * box_height + (box_height / 2), MIN(box_width, box_height) * 0.40, yellow);
    } else {
        printf("ERROR: Invalid player\n");
    }
}

// helper function for raising integers to a power
int int_pow(int base, int exp) {
    int result = 1;
    while (exp) {
        if (exp % 2)
            result *= base;
        exp /= 2;
        base *= base;
    }
    return result;
}

// function to update our game state data structure
void updateGameState(int *game_state_row, int row_to_place, int column_to_place, int player) {
    int prevGameState = game_state_row[row_to_place];
    int exponent = 2 * column_to_place + player - 1;
    game_state_row[row_to_place] = prevGameState + int_pow(2, exponent);
}

// function to checks if the game is over
// returns 0 if game not over, 1 if game over and the player who just played won, 2 if game is a tie
int checkGameOver(int *game_state_row, int *row_to_place, int player_who_just_played) {
    int i, j;
    int game_over = 0;
    int player_array[NUM_ROWS][NUM_COLS];

    // Check if player_who_just_played won by 4 in a row
    int intermediate_row = 0;
    for (i = 0; i < NUM_ROWS; i++) {
        for (j = 0; j < (NUM_COLS - 3); j++) {
            intermediate_row = (game_state_row[i] >> 2 * j) % 256;
            if (player_who_just_played == 1) {
                if (intermediate_row == 85) {
                    game_over = 1;
                    return game_over;
                }
            } else if (player_who_just_played == 2) {
                if (intermediate_row == 170) {
                    game_over = 1;
                    return game_over;
                }
            } else {
                printf("Player who just played is not 1 or 2! Error case.\n");
            }
        }
    }

    // Check if player_who_just_played won by 4 in a column
    int vertical_tracker[4];
    for (i = 0; i < (NUM_ROWS - 3); i++) {
        for (j = 0; j < NUM_COLS; j++) {
            vertical_tracker[0] = (game_state_row[i] >> 2 * j) % 4;
            vertical_tracker[1] = (game_state_row[i + 1] >> 2 * j) % 4;
            vertical_tracker[2] = (game_state_row[i + 2] >> 2 * j) % 4;
            vertical_tracker[3] = (game_state_row[i + 3] >> 2 * j) % 4;
            if (player_who_just_played == 1) {
                if ((vertical_tracker[0] == 1) && (vertical_tracker[1] == 1) && (vertical_tracker[2] == 1) && (vertical_tracker[3] == 1)) {
                    game_over = 1;
                    return game_over;
                }
            } else if (player_who_just_played == 2) {
                if ((vertical_tracker[0] == 2) && (vertical_tracker[1] == 2) && (vertical_tracker[2] == 2) && (vertical_tracker[3] == 2)) {
                    game_over = 1;
                    return game_over;
                }
            } else {
                printf("Player who just played is not 1 or 2! Error case.\n");
            }
        }
    }

    // Check if player_who_just_played won by 4 in a diagonal (lower left to upper right)
    int llur_diag_tracker[4];
    for (i = 0; i < (NUM_ROWS - 3); i++) {
        for (j = 3; j < NUM_COLS; j++) {
            llur_diag_tracker[0] = (game_state_row[i] >> 2 * j) % 4;
            llur_diag_tracker[1] = (game_state_row[i + 1] >> 2 * (j - 1)) % 4;
            llur_diag_tracker[2] = (game_state_row[i + 2] >> 2 * (j - 2)) % 4;
            llur_diag_tracker[3] = (game_state_row[i + 3] >> 2 * (j - 3)) % 4;
            if (player_who_just_played == 1) {
                if ((llur_diag_tracker[0] == 1) && (llur_diag_tracker[1] == 1) && (llur_diag_tracker[2] == 1) && (llur_diag_tracker[3] == 1)) {
                    game_over = 1;
                    return game_over;
                }
            } else if (player_who_just_played == 2) {
                if ((llur_diag_tracker[0] == 2) && (llur_diag_tracker[1] == 2) && (llur_diag_tracker[2] == 2) && (llur_diag_tracker[3] == 2)) {
                    game_over = 1;
                    return game_over;
                }
            } else {
                printf("Player who just played is not 1 or 2! Error case.\n");
            }
        }
    }

    // Check if player_who_just_played won by 4 in a diagonal (lower right to upper left)
    int lrul_diag_tracker[4];
    for (i = 0; i < (NUM_ROWS - 3); i++) {
        for (j = 0; j < (NUM_COLS - 3); j++) {
            lrul_diag_tracker[0] = (game_state_row[i] >> 2 * j) % 4;
            lrul_diag_tracker[1] = (game_state_row[i + 1] >> 2 * (j + 1)) % 4;
            lrul_diag_tracker[2] = (game_state_row[i + 2] >> 2 * (j + 2)) % 4;
            lrul_diag_tracker[3] = (game_state_row[i + 3] >> 2 * (j + 3)) % 4;
            if (player_who_just_played == 1) {
                if ((lrul_diag_tracker[0] == 1) && (lrul_diag_tracker[1] == 1) && (lrul_diag_tracker[2] == 1) && (lrul_diag_tracker[3] == 1)) {
                    game_over = 1;
                    return game_over;
                }
            } else if (player_who_just_played == 2) {
                if ((lrul_diag_tracker[0] == 2) && (lrul_diag_tracker[1] == 2) && (lrul_diag_tracker[2] == 2) && (lrul_diag_tracker[3] == 2)) {
                    game_over = 1;
                    return game_over;
                }
            } else {
                printf("Player who just played is not 1 or 2! Error case.\n");
            }
        }
    }

    // If no one has won yet, check if the board is not full
    for (i = 0; i < NUM_COLS; i++) {
        if (row_to_place[i] < NUM_ROWS) {
            game_over = 0;
            return game_over;
        }
    }

    // Game ended in a tie
    game_over = 2;
    return game_over;
}

// function that pulses hps-->fpga reset signal via PIO port
void pullReset() {
    *(reset_ptr) = 0;
    for (resetStall = 0; resetStall < 10000000; resetStall++) {}
    *(reset_ptr) = 1;
    for (resetStall = 0; resetStall < 10000000; resetStall++) {}
    *(reset_ptr) = 0;
}

int main(void) {
    // Open /dev/mem
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // Map memory
    h2p_lw_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (h2p_lw_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap1() failed...\n");
        close(fd);
        return 1;
    }

    vga_char_virtual_base = mmap(NULL, FPGA_CHAR_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_CHAR_BASE);
    if (vga_char_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap2() failed...\n");
        close(fd);
        return 1;
    }
    vga_char_ptr = (unsigned int *)(vga_char_virtual_base);

    vga_pixel_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, SDRAM_BASE);
    if (vga_pixel_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap3() failed...\n");
        close(fd);
        return 1;
    }
    vga_pixel_ptr = (unsigned int *)(vga_pixel_virtual_base);

    sram_virtual_base = mmap(NULL, FPGA_ONCHIP_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_ONCHIP_BASE);
    if (sram_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap4() failed...\n");
        close(fd);
        return 1;
    }
    sram_ptr = (unsigned int *)(sram_virtual_base);

    h2p_virtual_base = mmap(NULL, FPGA_AXI_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, FPGA_AXI_BASE);
    if (h2p_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap5() failed...\n");
        close(fd);
        return 1;
    }
    reset_ptr = (signed int *)(h2p_virtual_base + FPGA_RESET_BASE);

    char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
    char text_bottom_row[40] = "Graphics primitives\0";
    char color_index = 0;
    int color_counter = 0;

    // clear the screen
    VGA_box(0, 0, 640, 480, blue);
    // clear the text
    VGA_text_clear();

    // initialization of clock loop variable
    int clockStall = 0;

    int i;
    int j;

    int box_width = 640 / NUM_COLS;
    int box_height = 480 / NUM_ROWS;

    int game_state_row[NUM_ROWS];
    int row_to_place[NUM_COLS];

    char go_first;
    int player_move;
    int valid_move;
    int ai_move;

    int game_over;
    int winner;
    char play_again;

    do {
        // Initialize the variables
        valid_move = 1;
        game_over = 0;
        winner = -1;

        // Draw Empty Board
        for (i = 1; i < NUM_COLS; i++) {
            VGA_Vline(i * box_width, 0, 480, dark_blue);
        }

        for (i = 1; i < NUM_ROWS; i++) {
            VGA_Hline(0, i * box_height, 640, dark_blue);
        }

        for (i = 0; i < NUM_COLS; i++) {
            for (j = 0; j < NUM_ROWS; j++) {
                VGA_disc(i * box_width + (box_width / 2), j * box_height + (box_height / 2), MIN(box_width, box_height) * 0.40, black);
            }
        }

        // Declare Initial Game States
        for (i = 0; i < NUM_ROWS; i++) {
            game_state_row[i] = 0;
        }

        // Declare Initial Row to Place
        for (i = 0; i < NUM_COLS; i++) {
            row_to_place[i] = 0;
        }

        // Ask if player wishes to go first
        printf("Would you like to go first? [y/n]\n");
        scanf(" %c", &go_first);

        // Player wishes to go first
        if (go_first == 'y') {
            printf("Which column would you like to place your token in? [1-7]\n");
            scanf("%d", &player_move);
            player_move--;

            // Check if invalid move, prompt for a valid move
            if (row_to_place[player_move] >= NUM_ROWS || player_move < 0 || player_move > NUM_COLS) {
                valid_move = 0;
                do {
                    printf("Invalid move! That column is already full.\n");
                    printf("Which column would you like to place your token in? [1-7]\n");
                    scanf("%d", &player_move);
                    player_move--;
                    if (row_to_place[player_move] < NUM_ROWS && player_move > -1 && player_move < NUM_COLS) valid_move = 1;
                } while (!valid_move);
            }

            placeDisc(player_move, row_to_place[player_move], 2);
            // Update game state and row tracker variables
            updateGameState(game_state_row, row_to_place[player_move], player_move, 2);
            row_to_place[player_move]++;
        }

        pullReset();
        // Loop forever, interfacing with SRAM to play a game
        do {
            // Send game states for all rows
            for (i = 1; i < NUM_ROWS + 1; i++) {
                *(sram_ptr + i) = game_state_row[i - 1];
            }

            // Set the "data-ready" flag so the FPGA knows it's its turn to move
            *(sram_ptr) = 1;

            // Wait for the FPGA to zero the "data_ready" flag
            while (*(sram_ptr) == 1);

            // Read the AI's move
            ai_move = *(sram_ptr + NUM_ROWS + 1);
            printf("AI wants to place at column: %d\n", ai_move + 1);
            placeDisc(ai_move, row_to_place[ai_move], 1);

            // Update game state and row tracker variables
            updateGameState(game_state_row, row_to_place[ai_move], ai_move, 1);
            row_to_place[ai_move]++;

            // Check if the game is over
            game_over = checkGameOver(game_state_row, row_to_place, 1);
            if (game_over == 1) {
                winner = 1;
                break;
            } else if (game_over == 2) {
                winner = 0;
                break;
            }

            // Player's Move
            printf("Which column would you like to place your token in? [1-7]\n");
            scanf("%d", &player_move);
            player_move--;

            // Check if invalid move, prompt for a valid move
            if (row_to_place[player_move] >= NUM_ROWS || player_move < 0 || player_move > NUM_COLS) {
                valid_move = 0;
                do {
                    printf("Invalid move! That column is already full.\n");
                    printf("Which column would you like to place your token in? [1-7]\n");
                    scanf("%d", &player_move);
                    player_move--;
                    if (row_to_place[player_move] < NUM_ROWS && player_move > -1 && player_move < NUM_COLS) valid_move = 1;
                } while (!valid_move);
            }

            placeDisc(player_move, row_to_place[player_move], 2);
            // Update game state and row tracker variables
            updateGameState(game_state_row, row_to_place[player_move], player_move, 2);
            row_to_place[player_move]++;

            // Check if the game is over
            game_over = checkGameOver(game_state_row, row_to_place, 2);
            if (game_over == 1) {
                winner = 2;
                break;
            } else if (game_over == 2) {
                winner = 0;
                break;
            }
        } while (game_over == 0);

        if (winner == 1) {
            printf("AI wins! Better luck next time!\n");
        } else if (winner == 2) {
            printf("Congratulations, you win!\n");
        } else if (winner == 0) {
            printf("It's a draw! Not bad, but you can do better.\n");
        } else {
            printf("Error determining the winner.\n");
        }

        // Play again logic
        printf("Would you like to play again? [y/n]\n");
        scanf(" %c", &play_again);
    } while (play_again == 'y');
} // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char *text_ptr) {
    volatile char *character_buffer = (char *)vga_char_ptr;
    int offset;
    offset = (y << 7) + x;
    while (*(text_ptr)) {
        *(character_buffer + offset) = *(text_ptr);
        ++text_ptr;
        ++offset;
    }
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear() {
    volatile char *character_buffer = (char *)vga_char_ptr;
    int offset, x, y;
    for (x = 0; x < 79; x++) {
        for (y = 0; y < 59; y++) {
            offset = (y << 7) + x;
            *(character_buffer + offset) = ' ';
        }
    }
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X, Y) do { int temp = X; X = Y; Y = temp; } while (0)

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color) {
    int row, col;
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    if (y1 > y2) SWAP(y1, y2);
    for (row = y1; row <= y2; row++) {
        for (col = x1; col <= x2; ++col) {
            VGA_PIXEL(col, row, pixel_color);
        }
    }
}

/****************************************************************************************
 * Draw an outline rectangle on the VGA monitor 
****************************************************************************************/
void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color) {
    int row, col;
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    if (y1 > y2) SWAP(y1, y2);
    col = x1;
    for (row = y1; row <= y2; row++) {
        VGA_PIXEL(col, row, pixel_color);
    }
    col = x2;
    for (row = y1; row <= y2; row++) {
        VGA_PIXEL(col, row, pixel_color);
    }
    row = y1;
    for (col = x1; col <= x2; ++col) {
        VGA_PIXEL(col, row, pixel_color);
    }
    row = y2;
    for (col = x1; col <= x2; ++col) {
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a horizontal line on the VGA monitor 
****************************************************************************************/
void VGA_Hline(int x1, int y1, int x2, short pixel_color) {
    int col;
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (x1 > x2) SWAP(x1, x2);
    for (col = x1; col <= x2; ++col) {
        VGA_PIXEL(col, y1, pixel_color);
    }
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor 
****************************************************************************************/
void VGA_Vline(int x1, int y1, int y2, short pixel_color) {
    int row;
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (y2 < 0) y2 = 0;
    if (y1 > y2) SWAP(y1, y2);
    for (row = y1; row <= y2; row++) {
        VGA_PIXEL(x1, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/
void VGA_disc(int x, int y, int r, short pixel_color) {
    int row, col, rsqr, xc, yc;
    rsqr = r * r;
    for (yc = -r; yc <= r; yc++) {
        for (xc = -r; xc <= r; xc++) {
            col = xc;
            row = yc;
            if (col * col + row * row <= rsqr + r) {
                col += x;
                row += y;
                if (col > 639) col = 639;
                if (row > 479) row = 479;
                if (col < 0) col = 0;
                if (row < 0) row = 0;
                VGA_PIXEL(col, row, pixel_color);
            }
        }
    }
}

/****************************************************************************************
 * Draw a circle on the VGA monitor 
****************************************************************************************/
void VGA_circle(int x, int y, int r, int pixel_color) {
    int row, col, rsqr, xc, yc;
    int col1, row1;
    rsqr = r * r;
    for (yc = -r; yc <= r; yc++) {
        col1 = (int)sqrt((float)(rsqr + r - yc * yc));
        col = col1 + x;
        row = yc + y;
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        VGA_PIXEL(col, row, pixel_color);
        col = -col1 + x;
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        VGA_PIXEL(col, row, pixel_color);
    }
    for (xc = -r; xc <= r; xc++) {
        row1 = (int)sqrt((float)(rsqr + r - xc * xc));
        col = xc + x;
        row = row1 + y;
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        VGA_PIXEL(col, row, pixel_color);
        row = -row1 + y;
        if (col > 639) col = 639;
        if (row > 479) row = 479;
        if (col < 0) col = 0;
        if (row < 0) row = 0;
        VGA_PIXEL(col, row, pixel_color);
    }
}

/****************************************************************************************
 * Draw a line on the VGA monitor 
****************************************************************************************/
void VGA_line(int x1, int y1, int x2, int y2, short c) {
    int e;
    signed int dx, dy, j, temp;
    signed int s1, s2, xchange;
    signed int x, y;
    if (x1 > 639) x1 = 639;
    if (y1 > 479) y1 = 479;
    if (x2 > 639) x2 = 639;
    if (y2 > 479) y2 = 479;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 < 0) x2 = 0;
    if (y2 < 0) y2 = 0;

    x = x1;
    y = y1;

    if (x2 < x1) {
        dx = x1 - x2;
        s1 = -1;
    } else if (x2 == x1) {
        dx = 0;
        s1 = 0;
    } else {
        dx = x2 - x1;
        s1 = 1;
    }

    if (y2 < y1) {
        dy = y1 - y2;
        s2 = -1;
    } else if (y2 == y1) {
        dy = 0;
        s2 = 0;
    } else {
        dy = y2 - y1;
        s2 = 1;
    }

    xchange = 0;

    if (dy > dx) {
        temp = dx;
        dx = dy;
        dy = temp;
        xchange = 1;
    }

    e = ((int)dy << 1) - dx;

    for (j = 0; j <= dx; j++) {
        VGA_PIXEL(x, y, c);

        if (e >= 0) {
            if (xchange == 1) x = x + s1;
            else y = y + s2;
            e = e - ((int)dx << 1);
        }

        if (xchange == 1) y = y + s2;
        else x = x + s1;

        e = e + ((int)dy << 1);
    }
}
