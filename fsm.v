module project(SW, LEDR, KEY, CLOCK_50, GPIO, HEX0, HEX1, HEX2, HEX4, HEX5, HEX6);
     input [17:0] SW;
     input [3:0] KEY;
     input CLOCK_50;
     output [7:0] HEX0;
     output [7:0] HEX1;
     output [7:0] HEX2;
     output [7:0] HEX4;
     output [7:0] HEX5;
     output [7:0] HEX6;
     output [17:0] LEDR;
     output [20:0] GPIO;
     wire [3:0] motor;
     wire [10:0] stack_val;
     assign GPIO[17:14] = motor;
     wire [10:0] curr_state;
     
     
     turret_module turret(
        .clk(CLOCK_50),
        .manual_clock(~KEY[0]),
        .manual_active(~KEY[1]),
        .activate(~KEY[2]),
        .switch_input(SW[10:0]),
        .reset_turret(~KEY[3]),
        .motor_pins(motor),
        .fire_signal(GPIO[12]),
        .state(LEDR[2:0]),
        .out(curr_state),
        .stack_clock(LEDR[3]),
        .stack_active(LEDR[4]),
        .fire_delay(LEDR[5]),
        .top_of_stack(stack_val)
        );
    
    // Display the current position of turret on the LEDs and on the HEXs
	  assign LEDR[17:7] = curr_state;
     hex_display least_sig(
	      .IN(curr_state[3:0]), 
			.OUT(HEX0)
			);
     hex_display mid_sig(
	      .IN(curr_state[7:4]), 
			.OUT(HEX1)
			);
     hex_display most_sig(
	      .IN(curr_state[10:7]),
			.OUT(HEX2)
			);
			
     // Display the top of the stack on the LEDs
     hex_display least_sig_stack(
	      .IN(stack_val[3:0]), 
			.OUT(HEX4)
			);
     hex_display mid_sig_stack(
	      .IN(stack_val[7:4]), 
			.OUT(HEX5)
			);
     hex_display most_sig_stack(
	      .IN(stack_val[10:8]),
			.OUT(HEX6)
			);
endmodule

module turret_module(
    input clk,
    input manual_clock,
    input manual_active,
    input activate,
    input [10:0] switch_input,
    input reset_turret,
    output [3:0] motor_pins,
    output fire_signal,
    output [2:0] state,
    output wire [10:0] out,
    output stack_clock,
    output stack_active,
    output fire_delay,
	  output wire [10:0] top_of_stack
    );
     
     
    assign fire_delay = delay_clock;
    assign out = rotation;
	  assign top_of_stack = top_stack;
    wire [31:0] counterout;
    wire delay_clock;
    wire [10:0] stack_val;
    wire [10:0] rotation;
    wire [10:0] top_stack;
    wire reset_delay, reset_stack;
    wire movement_dir;
    wire move;
    // FSM 
    control tur_cont(
        .clk(clk),
        .resetn(reset_turret),
        .fire_delay(delay_clock),
        .rotation(rotation),
        .top_of_stack(top_stack),
        .activate(activate),
        .reset_delay(reset_delay),
        .reset_stack(reset_stack),
        .stack_mode(stack_mode),
        .stack_activate(stack_activate),
        .direction(movement_dir),
        .firing(fire_signal),
        .move(move),
        .manual_stack(manual_mode),
        .state(state)
    );
    
    datapath turret_data(
        .manual_clock(manual_clock),
        .manual_active(manual_active),
        .manual_mode(manual_mode),
        .clk(clk),
        .reset_stack(reset_stack | reset_turret),
        .data_in(switch_input),
        .ld_stack(stack_activate),
        .stack_mode(stack_mode),
        .reset_clock(reset_delay),
        .direction(movement_dir),
        .go(move),
        .motor_pins(motor_pins),
        .top_stack(top_stack),
        .clock_val(delay_clock),
        .position(rotation),
        .stack_clock(stack_clock),
        .stack_activate(stack_active),
        .counterout(counterout)
    );
endmodule
     

module motor(clock, stepperPins, direction, go, currPos);
 
parameter STEPPER_DIVIDER = 50000; // every 100ms
input direction;                  // Which way to turn the motor
input go;                         // Turn on the motor
input clock;
output reg [3:0] stepperPins;     // Stepper motor output
output reg [10:0] currPos;        // Current rotation of the motor

reg [31:0] clockCount;            // Counter to tell how fast the motor should run
reg [2:0] step;                   // 8 positions for half step

always @ (posedge clock)
begin
    // every millisecond, increase step
    if(clockCount >= STEPPER_DIVIDER && go == 1'b1)
        begin
		    // check which direction and move the motor, and update position accordingly 
			   if(direction == 1'b1)
                begin
					     // Move Left
                    step <= step + 1'b1;
                    currPos <= currPos + 1'b1;
                end
                else
                begin
					     // Move Right
                    step <= step - 1'b1;
                    currPos <= currPos - 1'b1;
                end
                clockCount <= 1'b0;
        end
    else
        clockCount <= clockCount + 1'b1;
end
 // SOURCE: https://trandi.wordpress.com/2014/09/17/fpga-rc-servo-and-stepper-motor
 // assigns motor stepper values to activate the motor.
 // assign motor val based on steppers
always @ (step)
begin
    case(step)
        0: stepperPins <= 4'b1000;
        1: stepperPins <= 4'b1100;
        2: stepperPins <= 4'b0100;
        3: stepperPins <= 4'b0110;
        4: stepperPins <= 4'b0010;
        5: stepperPins <= 4'b0011;
        6: stepperPins <= 4'b0001;
        7: stepperPins <= 4'b1001;
    endcase
end
 
endmodule

module control(
    input clk,
    input resetn,
    input fire_delay,
    input [10:0] rotation,
    input [10:0] top_of_stack,
    input activate,
    output reg reset_delay, reset_stack,
    output reg stack_mode, stack_activate,
    output reg direction,
    output reg firing,
    output reg move,
    output reg manual_stack,
    output [2:0] state
    );

    assign state = current_state;
     
    reg [2:0] current_state, next_state; 
    
    localparam  START           = 0,
                FIRE_LOOP       = 1,
                POP             = 2,
                FIRE            = 3,
                RESET           = 4,
                MAX_ROT         = 2000,
                MIN_ROT         = 10;
    // Next state logic aka our state table
    always@(*)
    begin: state_table 
            case (current_state)
                START:
                begin
					          // Activate is a key press that enables the turret loop 
                    if(activate == 1'b1)
                        next_state <= FIRE_LOOP; 
                    else // If we don't want to start the loop, stay in
                        next_state <= START;
                end
                // Start moving around to shoot targets
                FIRE_LOOP:
                begin 
                    // If the turret can still move
                    if (rotation < MAX_ROT)
                    begin
                        // Check if we've passed any position on the stack, and fire if we have
                        if(top_of_stack < rotation)
                            next_state <= POP;
                        // Otherwise, just keep moving
                        else
                            next_state <= FIRE_LOOP;
                    end
                    else
                        // If it can't move resest the turret
                        next_state <= RESET;
                    end
                POP:
                begin
                    // Pop the stack, and fire at the target
                    next_state <= FIRE;
                end
					      FIRE:
                begin
                    // We "fire" for a fixed amount of time
                    if (fire_delay == 1'b1)
                        next_state <= FIRE_LOOP;
                    else
                        next_state <= FIRE;
                end
                RESET:
                begin
                    // While the turret hasn't reached it's original position, keep moving it back
                    if (rotation  < MIN_ROT)
                        next_state <= START;
                    else
                        next_state <= RESET;
                end
                default:
                        next_state <= START;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(*)
    begin: enable_signals
        // Delay should only be incrementing while in a delay state.
        case (current_state)
            START:
            begin
                // Set the stack to push mode to put things on
                // as well as setting it to manual so that input will get
                // fed to the stack
                reset_delay      <= 1'b1;
                reset_stack      <= 1'b0;
                stack_mode       <= 1'b0;
                stack_activate   <= 1'b0;
                manual_stack     <= 1'b1;
                direction        <= 1'b1;
                move             <= 1'b0;
                firing           <= 1'b0;
            end
            FIRE_LOOP:
            begin
                // The turret begins scanning, turns and constantly checks the stack
                reset_delay      <= 1'b1;
                reset_stack      <= 1'b0;
                stack_mode       <= 1'b1;
                stack_activate   <= 1'b0;
                manual_stack     <= 1'b0;
                direction        <= 1'b1;
                move             <= 1'b1;
                firing           <= 1'b0;
            end
            POP:
            begin
                // Pops the seen value from the stack
                // Stops resetting the delay counter so that
                // the firing time is timed
                reset_delay      <= 1'b0;
                reset_stack      <= 1'b0;
                stack_mode       <= 1'b1;
                stack_activate   <= 1'b1;
                manual_stack     <= 1'b0;
                direction        <= 1'b1;
                move             <= 1'b0;
                firing           <= 1'b0;
            end
            FIRE:
            begin
                // Enable fire signal, turn off movement
                reset_delay      <= 1'b0;
                reset_stack      <= 1'b0;
                stack_mode       <= 1'b1;
                stack_activate   <= 1'b0;
                manual_stack     <= 1'b0;
                direction        <= 1'b1;
                move             <= 1'b0;
                firing           <= 1'b1;
            end
            RESET:
            begin
                // Reverse movement, reset back to original position
                reset_delay      <= 1'b1;
                reset_stack      <= 1'b1;
                stack_mode       <= 1'b1;
                stack_activate   <= 1'b0;
                manual_stack     <= 1'b0;
                direction        <= 1'b0;
                move             <= 1'b1;
                firing           <= 1'b0;
            end
            default:
            begin
                reset_delay      <= 1'b1;
                reset_stack      <= 1'b0;
                stack_mode       <= 1'b1;
                stack_activate   <= 1'b0;
                manual_stack     <= 1'b0;
                direction        <= 1'b0;
                move             <= 1'b0;
                firing           <= 1'b0;
            end
        endcase
    end // enable_signals
   
    // current_state registers
    always@(posedge clk)
    begin: state_FFs
        if(resetn)
            current_state <= START;
        else
            current_state <= next_state;
    end
endmodule


module datapath(
    input manual_clock,
    input manual_active,
    input manual_mode,
    input clk,
    input reset_stack,
    input [10:0] data_in,
    input ld_stack, 
    input stack_mode,
    input reset_clock,
    input direction,
    input go,
    output [3:0] motor_pins,
    output [10:0] top_stack,
    output clock_val,
    output [10:0] position,
    output wire stack_clock,
    output wire stack_activate,
    output wire [31:0] counterout
    );
     
    //wire stack_activate;
    //wire stack_clock;
     
     // Mux to feed the stack a manual activation signal or automatic activation signal
     mux2to1 active_mux(
        .val0(ld_stack),
        .val1(manual_active),
        .select(manual_mode),
        .out(stack_activate)
        );
    
    // Mux to feed manunal or automatic clock signals
    mux2to1 clock_mux(
        .val0(clk),
        .val1(manual_clock),
        .select(manual_mode),
        .out(stack_clock)
        );
        
    // The stack of position values to shoot at
    // it has a "height" of 8 registers, and width of 11 bits of storage
    stack8h11b stack(
        .load_val(data_in),
        .mode(stack_mode),
        .active(stack_activate),
        .reset(reset_stack),
        .clock(stack_clock),
        .out(top_stack)
        ); 
        
    // The counter for the amount of firing time
    rate_divider(
        .clock(clk),
        .out(clock_val),
        .reset(reset_clock),
        .counterout(counterout)
    );
    
    // Motor turning module
    motor(
        .clock(clk), 
        .stepperPins(motor_pins), 
        .direction(direction), 
        .go(go), 
        .currPos(position)
        );

endmodule


module stack8h11b(load_val, mode, active, reset, clock, out);
    input [10:0] load_val;
    input mode, active, reset, clock;
    output [10:0] out;
    // PUSH: mode 0
    // POP: mode 1
    // active: only push/pop when active is true

    // Declare eight 11-bit wires
    wire [10:0] register_out_0;
    wire [10:0] register_out_1;
    wire [10:0] register_out_2;
    wire [10:0] register_out_3;
    wire [10:0] register_out_4;
    wire [10:0] register_out_5;
    wire [10:0] register_out_6;
    wire [10:0] register_out_7;

    // Return the top of the stack
    assign out = register_out_0;
    // The stack registers
    stack_store block0(
        .push(load_val),
        .pop(register_out_1),
        .mode(mode),
        .active(active),
        .store(register_out_0),
        .reset(reset),
        .clock(clock)
    );
    stack_store block1(
        .push(register_out_0),
        .pop(register_out_2),
        .mode(mode),
        .active(active),
        .store(register_out_1),
        .reset(reset),
        .clock(clock)
    );
    stack_store block2(
        .push(register_out_1),
        .pop(register_out_3),
        .mode(mode),
        .active(active),
        .store(register_out_2),
        .reset(reset),
        .clock(clock)
    );
    stack_store block3(
        .push(register_out_2),
        .pop(register_out_4),
        .mode(mode),
        .active(active),
        .store(register_out_3),
        .reset(reset),
        .clock(clock)
    );
    stack_store block4(
        .push(register_out_3),
        .pop(register_out_5),
        .mode(mode),
        .active(active),
        .store(register_out_4),
        .reset(reset),
        .clock(clock)
    );
    stack_store block5(
        .push(register_out_4),
        .pop(register_out_6),
        .mode(mode),
        .active(active),
        .store(register_out_5),
        .reset(reset),
        .clock(clock)
    );
    stack_store block6(
        .push(register_out_5),
        .pop(register_out_7),
        .mode(mode),
        .active(active),
        .store(register_out_6),
        .reset(reset),
        .clock(clock)
    );
    wire [10:0] default_end_val;
    // Feed the end of the stack a "NULL" value which simply represents that the stack is position is empty
    assign default_end_val = 11'b11111111111;
    stack_store block7(
        .push(register_out_6),
        .pop(default_end_val),
        .mode(mode),
        .active(active),
        .store(register_out_7),
        .reset(reset),
        .clock(clock)
    );
endmodule

module stack_store(push, pop, mode, active, store, reset, clock);
    input [10:0] push, pop;
    input mode, active, reset, clock;
    output [10:0] store;

    wire [10:0] load_val;

    // Decide whether to load value from
    // above or below in stack based on
    // if mode is push or pull
    mux11bit2to1 mux0(
        .push(push),
        .pop(pop),
        .mode(mode),
        .out(load_val)
    );

    // Storage for one block of the stack
    reg8bit register0(
        .load_val(load_val),
        .load_n(active),
        .reset(reset),
        .clock(clock), 
        .storage(store)
    );
endmodule

module mux11bit2to1(push, pop, mode, out);
    input [10:0] push; //selected when mode is 0
    input [10:0] pop; //selected when mode is 1
    input mode; //select signal
    output reg [10:0] out; //output
    
    always@(*)
    begin: mux
        out = mode ? pop : push;
    end 
endmodule

module mux2to1(val0, val1, select, out);
    input val0; //selected when mode is 0
    input val1; //selected when mode is 1
    input select; //select signal
    output reg out; //output
    
    always@(*)
    begin: mux
        out = select ? val1 : val0;
    end 
endmodule

module reg8bit(load_val, load_n, reset, clock, storage);
    input [10:0] load_val;
    input reset, clock, load_n;
    output reg [10:0] storage;
    
    always @(posedge clock)
    begin
        if(reset == 1'b1)
            storage <= 11'b11111111111;
        else if (load_n == 1'b1)
            storage <= load_val;
    end
endmodule


module rate_divider(clock, out, reset, counterout);
    input clock;
    input reset;
    reg [31:0] counter; //2hz
    output out;
    output [31:0] counterout;
    assign counterout = counter;
    localparam  DELAY = 32'h00ffffff;
    assign out = (counter > DELAY);
    
    
    
    always @(posedge clock)
    begin
        if(reset == 1'b0)
            begin
            if(counter == 32'hffffffff)
                counter <= 0;
              else
                counter <= counter + 1'b1;
            end
        else
            counter <= 0;
    end
endmodule

module hex_display(IN, OUT);
    input [3:0] IN;
	 output reg [7:0] OUT;
	 
	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;
			
			default: OUT = 7'b0111111;
		endcase

	end
endmodule
