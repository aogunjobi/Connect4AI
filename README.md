# Connect4AI

Welcome to the **Connect4AI** repository! This project implements an AI-powered version of the classic Connect 4 game with FPGA and VGA display support. The AI uses advanced algorithms to provide a challenging and enjoyable gaming experience.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Project Files](#project-files)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction

Connect4AI is an AI-driven implementation of the Connect 4 game. The goal is to connect four of your pieces in a row, either horizontally, vertically, or diagonally, before your opponent does. This project leverages AI and FPGA to create a competitive opponent that can adapt and strategize effectively, with a VGA display for visual representation.

## Features

- **AI Strategies**: The AI uses defensive strategies to make intelligent moves.
- **FPGA Integration**: Utilizes FPGA for game logic and AI processing.
- **VGA Display**: Provides a visual interface for the game.

## Requirements

- FPGA Development Board (e.g., DE1-SoC)
- VGA Monitor
- Necessary FPGA tools and software

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/aogunjobi/Connect4AI.git
   cd Connect4AI
   ```

2. Follow the instructions in the project files to set up the FPGA and VGA display.

## Usage

1. Compile the provided Verilog files using your FPGA development environment.
2. Upload the compiled code to your FPGA board.
3. Connect the VGA monitor to the FPGA board.
4. Follow the on-screen instructions to start playing.

## Project Files

- **Connect Four Interface with VGA Display.c**: C code for interfacing with the VGA display.
- **connectFourAI_defensive.v**: Verilog module implementing the AI's defensive strategy.
- **ConnectFourAI_TopModule.v**: Top module for integrating the Connect 4 game and AI.
- **DE1_SoC_FPGA_HPS_Interface_Connect_Four.v**: Verilog module for interfacing the FPGA with the HPS (Hard Processor System) on the DE1-SoC board.
