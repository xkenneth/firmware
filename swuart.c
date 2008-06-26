// ------------------------------------------------------------------------
// Implements a software UART.
//
// This software-based UART samples the incoming RX signal at 3-times 
// the baud rate and assembles the bits into 8-bit words.
// Rev 01	02/02/06	Increase data buffers to 20 bytes
// Rev 02	08/16/06	Added PWM output to UARTServ() interrupt handler
// Rev 03   12/01/06	Change PWM to motor position controller
// Rev 04	02/02/07	Use a 4 bit counter for PWM instead of a 6 bit one
// Rev 05	02/13/07	FL0 is now used as BRAKE, RS232 ENABLE is connected to GND 
// ------------------------------------------------------------------------

#include    "swuart.h"
#include	"RS232.h"

incasm("def2181.h");

// --------------------------------------------------------------
// These macros should be changed to fit the specific hardware.
// 
// SetTX should set the RS-232 value high.
// ClrTX should set the RS-232 value low.
// GetRX should return non-zero for a HIGH RS-232 value.
// EnableRS232 should set digital switch enable low.
// --------------------------------------------------------------
#define     SetTX()       asm(" SET FLAG_OUT;")
#define     ClrTX()       asm(" RESET FLAG_OUT;")
#define     GetRX(v)      asm(" ar=0;if NOT FLAG_IN jump n0_rxval;ar=1;n0_rxval:" : "=c" (v) ::)   
//#define     EnableRS232() asm(" RESET FL0;");

//#define     RX_QUE_SIZE     20
//#define     TX_QUE_SIZE     20
#define     RX_QUE_SIZE     100
#define     TX_QUE_SIZE     100
#define     NUM_DATA_BITS   8

// ------------------------------------
// Local Data
// ------------------------------------
static uchar            rx_que[RX_QUE_SIZE];
static volatile uchar   rx_que_in_idx=0;
static volatile uchar   rx_que_out_idx=0;
static ushort           rx_bit_count=0;

static uchar            tx_que[TX_QUE_SIZE];
static volatile uchar   tx_que_in_idx=0;
static volatile uchar   tx_que_out_idx=0;
static ushort           tx_bit_count=0;

// ----------------------------------------------------------------
// Initializes the software UART. This must be called on power-up.
// ----------------------------------------------------------------
void UARTInit()
{
//    EnableRS232();
    SetTX();
}

extern unsigned short PWM_setting; //PWM to output for motor speed

void PWM_controller()
{

// assembler calls to control the line used for PWMout
// which controls the motor speed.
// the PWM is a 6 bit counter (0 to x3f) which counts continually
// the PWM_setting is the number of counts which the PWM oututs a high level
#define PWM_ON()		asm(" SET FL1;")
#define PWM_OFF()		asm(" RESET FL1;")
     
    static unsigned short PWM_counter = 0; //tick counter for PWM output
   
    // generate the PWM output
    if(PWM_counter < PWM_setting) PWM_OFF(); //PWM_ON();
    else PWM_ON(); //PWM_OFF();

    //increment the PWM counter then mask off bits above 63
    PWM_counter = (++PWM_counter & 0xf) ; 
    
}


extern short CMD_Counter;
extern bool CMD_ESC;

// -----------------------------------------------------------------------
// UARTServ
//
// This routine must be called at exactly 3 times the baud rate; usually 
// from a timer interrupt routine.
// -----------------------------------------------------------------------
//extern unsigned short PWM_setting = 0; //PWM to output for motor speed

void UARTServ()
{
    static  uchar   rx_char, tx_char;
    static  ushort  rx_slot_count=0, tx_slot_count=0;

    uchar   val;
      
    // ------------------------------------------------------------------
    // Process RX
    //
    // Only do RX processing when the slot counter
    // is zero. This causes sampling to occur near
    // the center of each bit period.
    // ------------------------------------------------------------------
    if(!rx_slot_count)
    {
        GetRX(val);         // Get RX bit value
        if(!rx_bit_count)
        {
            // ---------------------------------------------------------
            // Expecting START BIT.
            // If val==0, we have a start bit - increment the bit count
            // and clear the current rx charactor. If we don't have a
            // start bit, just return.
            // ---------------------------------------------------------
            if(!val)
            {
                ++rx_bit_count;
                rx_char = 0;
                rx_slot_count=3;
            }
        }
        else if(rx_bit_count>NUM_DATA_BITS)
        {
            // -------------------------------------------------------
            // Expecting a STOP BIT. If val==1, we have a stop bit - 
            // store the received char in the que and reset the bit
            // count.
            //
            // If this is not a stop bit, don't store the char. 
            // -------------------------------------------------------
            if(val)
            {
                switch (rx_char) {
                	
                	case CR:
                	
                		CMD_Counter++;
                		
                		rx_que[rx_que_in_idx++] = rx_char;
                		if(rx_que_in_idx >= RX_QUE_SIZE)
                    		rx_que_in_idx=0;
                		break;
                		
                	case ESC:
                	
                		if (CMD_Counter>0) CMD_ESC = true;
                		break;
                		
                	case BS:		//backspace
                	
                		if(rx_que_in_idx == 0) {
                			
                			if (rx_que[RX_QUE_SIZE-1]!=CR)		//only back to the last command
                			
                    			rx_que_in_idx=RX_QUE_SIZE-1;
                    			
                		}
                		
                    	else {
                    		
                    		if(rx_que[rx_que_in_idx-1]!=CR)
                    		
                    			rx_que_in_idx--;
                    			
                    	}
                    	
                    	break;
                    	
                    default:
                
            			rx_que[rx_que_in_idx++] = rx_char;
                		if(rx_que_in_idx >= RX_QUE_SIZE)
                    		rx_que_in_idx=0;
                    		
                }
            }
            rx_bit_count=0;
            rx_slot_count=0;
        }
        else 
        {
            // -------------------------------------------------
            // Expecting a DATA BIT. Shift a zero into the next
            // bit position and set the MSBit if val==1.
            // -------------------------------------------------
            ++rx_bit_count;
            rx_char >>= 1;
            if(val)
                rx_char |= 0x80;
            rx_slot_count=2;
        }
    }
    else
        --rx_slot_count;

    // --------------------------------------------
    // Process TX
    //
    // Only do TX processing when the slot counter
    // is zero. This causes the bit periods to be
    // timed correctly. 
    // --------------------------------------------
    if(!tx_slot_count)
    {
        if(tx_bit_count)
        {
            ++tx_bit_count;
            // ----------------------------------------------------
            // If this is the start bit, set the TX line to zero.
            // If this is the stop bit, set the TX line to one.
            // Else, set the line depending on the next bit in
            // "tx_char".
            // ----------------------------------------------------
            if(tx_bit_count==2)  
                ClrTX();            // Send start bit
            else if(tx_bit_count >= NUM_DATA_BITS+3)
            {
                SetTX();            // Send stop bit
                tx_bit_count=0;
            }
            else
            {
                if(tx_char & 0x1)
                    SetTX();
                else
                    ClrTX();
                tx_char >>= 1;
            }
            tx_slot_count=2;
        }
        else if(tx_que_out_idx != tx_que_in_idx)
        {
            tx_char = tx_que[tx_que_out_idx++];
            if(tx_que_out_idx >= TX_QUE_SIZE)
                tx_que_out_idx=0;
            tx_bit_count=1;
        }
    }
    else
        --tx_slot_count;

}

// ----------------------------------------------------------
// UARTRxReady
//
//  Returns: 0=There are no chars in the RX queue.
//           1=There's at least one char in the RX queue.
// ----------------------------------------------------------
ushort UARTRxReady()
{
    return (rx_que_in_idx==rx_que_out_idx) ? 0:1;
}

// ----------------------------------------------------------
// UARTGetChar
//
// Returns the next char from the RX queue.
// ----------------------------------------------------------
uchar UARTGetChar()
{
    uchar   val=0;

    if(UARTRxReady())
    {
        val = rx_que[rx_que_out_idx++];
        if(rx_que_out_idx >= RX_QUE_SIZE)
            rx_que_out_idx=0;
    }    
    return val;
}

// ----------------------------------------------------------
// UARTPutChar
//
// Puts a char in the TX que.
// ----------------------------------------------------------
void UARTPutChar(uchar val)
{
    ushort  next_idx = tx_que_in_idx+1;

    if(next_idx >= TX_QUE_SIZE)
        next_idx=0;

    while(next_idx == tx_que_out_idx);

    tx_que[tx_que_in_idx] = val;
    tx_que_in_idx = next_idx;
}

