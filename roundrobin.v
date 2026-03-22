//4 request Round Robin arbiter
/*once a request is granted the priority shifts, such that the request that just got granted is shifted to last priority 
and the adjacent request is now first priority*/

module roundrobin(req,gnt,Clk);
input [3:0] req;
output reg [3:0] gnt=4'b0000;
input Clk; 
reg [1:0] priority = 2'b00; //initialize the priority
// priority = 0 (R0 > R1 > R2 > R3) - default
// priority = 1 (R1 > R2 > R3 > R0)
// priority = 2 (R2 > R3 > R0 > R1)
// priority = 3 (R3 > R0 > R1 > R2)

/*example 1 : current priority = 0 (R0 > R1 > R2 > R3), and the inputs of the requests are : 0010 i.e R1 is high this means that
the new priority order should have R2 as top priority and R1 as least priority which is R2>R3>R0>R1 - corresponding to priority = 2*/
/*example 2 : current priority = 2 (R2 > R3 > R0 > R1), and the inputs of the requests are : 0011 i.e R0 and R1 are high, here     
since R0>R1, R0 will be granted and the priority shifts such that R1 is now top priority and R0 is least priority i.e (R1>R2>R3>R0) - corresponding to priority = 1*/
always @ (posedge Clk) begin
    
    case(priority)
    2'b00 : begin
        if(req[0]) begin
        gnt <= 4'b0001;
        priority <=2'b01;
        end

        else if(req[1]) begin
        gnt <= 4'b0010 ; 
        priority <= 2'b10;
        end

        else if(req[2]) begin
        gnt <= 4'b0100;
        priority <= 2'b11;
        end

        else if(req[3]) begin
        gnt <= 4'b1000;
        priority <= 2'b00;
        end
        
        else begin
        gnt <= 4'b0000;
        end 
    end

    2'b01 : begin
        if(req[1]) begin
            gnt <= 4'b0010 ; 
            priority <= 2'b10;
        end

        else if (req[2]) begin
            gnt <= 4'b0100;
            priority <= 2'b11;
        end

        else if (req[3]) begin
            gnt <= 4'b1000;
            priority <= 2'b00;
        end

        else if (req[0]) begin
            gnt <= 4'b0001;
            priority <= 2'b01;
        end
        
        else gnt <= 4'b0000;
    end

    2'b10 : begin
        if(req[2]) begin
            gnt <= 4'b0100;
            priority <= 2'b11;
        end

        else if(req[3]) begin
            gnt <= 4'b1000;
            priority <= 2'b00;
        end

        else if(req[0]) begin
            gnt <= 4'b0001;
            priority <= 2'b01;
        end

        else if(req[1]) begin
            gnt <= 4'b0010;
            priority <= 2'b10;
        end

        else gnt <= 4'b0000;
    end

    2'b11 : begin
        if(req[3]) begin
            gnt <= 4'b1000;
            priority <= 2'b00;
        end

        else if(req[0]) begin
            gnt <= 4'b0001;
            priority <= 2'b01;
        end

        else if(req[1]) begin
            gnt <= 4'b0010;
            priority <= 2'b10;
        end

        else if(req[2]) begin
            gnt <= 4'b0100;
            priority <= 2'b11;
        end

        else gnt <= 4'b0000;
    end
    endcase
   end


endmodule


`timescale 1ns/1ps
module roundrobin_tb;
//define the input and output signals
wire [3:0] pgnt;
reg [3:0] preq;
reg pClk;
roundrobin uutest (
        .req(preq),
        .gnt(pgnt),
        .Clk(pClk));

//clock generation
initial begin
    pClk = 0 ;
    
    forever #10 pClk = ~pClk; //clock toggles every 10ns hence the time period is 20ns
end

//feed the inputs
initial begin
preq = 4'b0000; #20
//test case 1
preq = 4'b0001; #20;
preq = 4'b0010; #20;
preq = 4'b0100; #20;
preq = 4'b1000; #20;

preq = 4'b0010; #60; //testing only one input for a long time

preq = 4'b1000; #20; // now the priority = 0 i.e at default state

preq = 4'b0111; #20;
preq = 4'b0011; #20;
$finish;
end
initial begin
$dumpfile("roundrobin.vcd");
$dumpvars(0, roundrobin_tb);
end

endmodule
