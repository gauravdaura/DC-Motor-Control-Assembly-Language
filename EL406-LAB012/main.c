//-----------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : MAIN.C
// FILE VERSION : 1.0
// PROGRAMMER   : Programmer_Name
//-----------------------------------------------------------------------------
// REVISION HISTORY
//-----------------------------------------------------------------------------
//
// 1.0, YYYY-MM-DD, Programmer_Name
// - Initial release
//
//-----------------------------------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// REGISTER BASE AND OFFSET DEFINITIONS
//-----------------------------------------------------------------------------

#define SYSCTL_BASE             0x400fe000
#define SYSCTL_O_RCC            0x060
#define SYSCTL_O_RCGCGPIO       0x608
#define SYSCTL_O_RCGCADC        0x638
#define SYSCTL_O_RCGCPWM        0x640

#define GPIO_PORTA_BASE         0x40004000
#define GPIO_PORTB_BASE         0x40005000
#define GPIO_PORTC_BASE         0x40006000
#define GPIO_PORTD_BASE         0x40007000
#define GPIO_PORTE_BASE         0x40024000
#define GPIO_PORTF_BASE         0x40025000
#define GPIO_O_DATA             0x3fc
#define GPIO_O_DIR              0x400
#define GPIO_O_AFSEL            0x420
#define GPIO_O_PUR              0x510
#define GPIO_O_DEN              0x51c
#define GPIO_O_AMSEL            0x528
#define GPIO_O_PCTL             0x52c

#define PWM0_BASE               0x40028000
#define PWM1_BASE               0x40029000
#define PWM_O_0_CTL             0x040
#define PWM_O_0_GENA            0x060
#define PWM_O_0_GENB            0x064
#define PWM_O_0_LOAD            0x050
#define PWM_O_0_CMPA            0x058
#define PWM_O_0_CMPB            0x05c
#define PWM_O_ENABLE            0x008

#define ADC0_BASE               0x40038000
#define ADC1_BASE               0x40039000
#define ADC_O_ACTSS             0x000
#define ADC_O_RIS               0x004
#define ADC_O_IM                0x008
#define ADC_O_ISC               0x00c
#define ADC_O_EMUX              0x014
#define ADC_O_PSSI              0x028
#define ADC_O_SAC               0x030
#define ADC_O_CTL               0x038
#define ADC_O_SSFIFO3           0x0a8
#define ADC_O_SSMUX3            0x0a0
#define ADC_O_SSCTL3            0x0a4

#define CORE_PERIPH_BASE        0xe000e000
#define CORE_PERIPH_O_STCTRL    0x010
#define CORE_PERIPH_O_STRELOAD  0x014
#define CORE_PERIPH_O_STCURRENT 0x018
#define CORE_PERIPH_O_NVIC_EN0  0x100

// Macro for 32-bit register access
#define HWREG( x ) ( *( ( volatile uint32_t * )( x ) ) )

//-----------------------------------------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------------------------------------

// System Flags (Events)
uint32_t g_uiSysFlags;

#define SYSF_TICK               0       // System Tick Interval
#define SYSF_ADC_DONE           1       // ADC Complete

// Heartbeat Counter
uint32_t g_uiHBCount;
#define HB_TGLDELAY             125-1

// ADC Sample Counter
uint32_t g_uiSampleCount;
#define PE3_SAMPLEDELAY         5-1

//-----------------------------------------------------------------------------
// FUNCTION : void SysTickHndlr( void )
// PURPOSE  : Handles the system timer interrupt
//-----------------------------------------------------------------------------

void SysTickHndlr( void )
{
    // If the COUNT bit is set, indicate so as a system flag (event)
    if( HWREG( CORE_PERIPH_BASE + CORE_PERIPH_O_STCTRL ) & ( 1 << 16 ) )
    {
        g_uiSysFlags |= ( 1 << SYSF_TICK );
    }

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void ADC0Hndlr( void )
// PURPOSE  : Handles the ADC0 interrupt
//-----------------------------------------------------------------------------

void ADC0S3Hndlr( void )
{
    // Acknowledge interrupt (clear)
    HWREG( ADC0_BASE + ADC_O_ISC ) = 0x08;

    // Indicate that the conversion is complete as a system flag (event)
    g_uiSysFlags |= ( 1 << SYSF_ADC_DONE );

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void UpdatePWM( uint32_t uiADCResult )
// PURPOSE  : Updates the PWM output at PB6 based on iADCResult
//-----------------------------------------------------------------------------

void UpdatePWM( uint32_t uiADCResult )
{
    // Determine direction
    bool bDirection = uiADCResult & 0x0800 ? true : false;

    // Adjust duty cycle (0-1023)
    uiADCResult = uiADCResult >= 0x0800 ? ~uiADCResult : uiADCResult;
    uiADCResult = ( uiADCResult >> 1 ) & 0x03fe;

    // Assign direction
    HWREG( GPIO_PORTA_BASE + ( 0x04 << 2 ) ) = bDirection ? 0x04 : 0x00;
    HWREG( GPIO_PORTA_BASE + ( 0x08 << 2 ) ) = bDirection ? 0x00 : 0x08;

    // Update the compare A level
    HWREG( PWM0_BASE + PWM_O_0_CMPA ) = uiADCResult;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : uint32_t ReadADCResult( void )
// PURPOSE  : Reads the quantized value from PE0
//-----------------------------------------------------------------------------

uint32_t ReadADCResult( void )
{
    // Return the latest converted result
    return ( HWREG( ADC0_BASE + ADC_O_SSFIFO3 ) & 0x0fff );
}

//-----------------------------------------------------------------------------
// FUNCTION : void StartADCConversion( void )
// PURPOSE  : Initiates an ADC conversion
//-----------------------------------------------------------------------------

void StartADCConversion( void )
{
    // Start a conversion
    HWREG( ADC0_BASE + ADC_O_PSSI ) = 0x08;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void SamplePE3( void )
// PURPOSE  : Samples PE3 every 5 ms
//-----------------------------------------------------------------------------

void SamplePE3( void )
{
    // Check if a conversion is required
    if( !g_uiSampleCount-- )
    {
        // Reload sample interval counter
        g_uiSampleCount = PE3_SAMPLEDELAY;

        // Initiate a conversion
        StartADCConversion();
    }

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void ControlBlueLED( void )
// PURPOSE  : Controls the on-board heartbeat LED (PF2)
//-----------------------------------------------------------------------------

void ControlBlueLED( void )
{
    // Check if the LED must be toggled
    if( !g_uiHBCount-- )
    {
        // Reload the heartbeat interval counter
        g_uiHBCount = HB_TGLDELAY;

        // Toggle the blue LED
        HWREG( GPIO_PORTF_BASE + GPIO_O_DATA ) ^= 0x04;
    }

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void InitSysTick( void )
// PURPOSE  : Initializes the system timer (tick)
//-----------------------------------------------------------------------------

void InitSysTick( void )
{
    // Clear the current register
    HWREG( CORE_PERIPH_BASE + CORE_PERIPH_O_STCURRENT ) = 0;

    // Load the reload register for a 1 ms tick interval
    HWREG( CORE_PERIPH_BASE + CORE_PERIPH_O_STRELOAD ) = 4000-1;

    // Enable the system timer with interrupt
    HWREG( CORE_PERIPH_BASE + CORE_PERIPH_O_STCTRL ) = 0x03;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void InitPWM( void )
// PURPOSE  : Initializes the PWM peripheral (PB6)
//-----------------------------------------------------------------------------

void InitPWM( void )
{
    // Enable run-mode clocks
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCPWM  ) |= 0x01;
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCGPIO ) |= 0x02;

    // Configure PB6 as a PWM signal
    HWREG( GPIO_PORTB_BASE + GPIO_O_DEN   ) = 0x40;
    HWREG( GPIO_PORTB_BASE + GPIO_O_AFSEL ) = 0x40;
    HWREG( GPIO_PORTB_BASE + GPIO_O_PCTL  ) &= ~( 0x0f << 24 );
    HWREG( GPIO_PORTB_BASE + GPIO_O_PCTL  ) |=  ( 0x04 << 24 );

    // Use system clock as PWM clock source (16 MHz)
    HWREG( SYSCTL_BASE + SYSCTL_O_RCC ) &= ~( 0x1e << 16 );

    // Drive pwmA high on LOAD and low when CMPA=COUNT (counting down)
    HWREG( PWM0_BASE + PWM_O_0_CTL  ) = 0;
    HWREG( PWM0_BASE + PWM_O_0_GENA ) = 0x008c;

    // Configure PWM frequency of 15.625 kHz
    HWREG( PWM0_BASE + PWM_O_0_LOAD ) = 1024-1;

    // Assign initial duty cycle of 0
    HWREG( PWM0_BASE + PWM_O_0_CMPA ) = 1024-1;

    // Enable PWM
    HWREG( PWM0_BASE + PWM_O_0_CTL  ) = 1;
    HWREG( PWM0_BASE + PWM_O_ENABLE ) = 3;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void InitADC( void )
// PURPOSE  : Initializes the analog-to-digital converter (PE3)
//-----------------------------------------------------------------------------

void InitADC( void )
{
    // Enable run-mode clocks
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCADC  ) |= 0x01;
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCGPIO ) |= 0x10;

    // Configure PE3 as an analog input (AIN0)
    HWREG( GPIO_PORTE_BASE + GPIO_O_AFSEL ) |= 0x08;
    HWREG( GPIO_PORTE_BASE + GPIO_O_DEN   ) |= 0x08;
    HWREG( GPIO_PORTE_BASE + GPIO_O_AMSEL ) |= 0x08;

    // Configure sample sequencer for AIN0
    HWREG( ADC0_BASE + ADC_O_SSCTL3 ) = 0x06;
    HWREG( ADC0_BASE + ADC_O_IM     ) |= 0x08;
    HWREG( ADC0_BASE + ADC_O_ACTSS  ) |= 0x08;

    // Enable 64-sample hardware averager
    HWREG( ADC0_BASE + ADC_O_SAC ) |= 0x06;

    // Enable interrupt
    HWREG( CORE_PERIPH_BASE + CORE_PERIPH_O_NVIC_EN0 ) |= ( 1 << 17 );

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void InitBlueLED( void )
// PURPOSE  : Initializes the on-board Blue LED (PF2)
//-----------------------------------------------------------------------------

void InitBlueLED( void )
{
    g_uiHBCount = 0;

    // Enable run-mode clock
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCGPIO ) |= 0x20;

    // Configure PF2 as digital output
    HWREG( GPIO_PORTF_BASE + GPIO_O_DEN  ) |= 0x04;
    HWREG( GPIO_PORTF_BASE + GPIO_O_DIR  ) |= 0x04;
    HWREG( GPIO_PORTF_BASE + GPIO_O_DATA ) &= ~0x04;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void InitMotorDirection( void )
// PURPOSE  : Initializes the motor direction signals (PA2/PA3)
//-----------------------------------------------------------------------------

void InitMotorDir( void )
{
    // Enable run-mode clock
    HWREG( SYSCTL_BASE + SYSCTL_O_RCGCGPIO ) |= 0x01;

    // Configure PA2 and PA3 as digital outputs
    HWREG( GPIO_PORTA_BASE + GPIO_O_DEN  ) |= 0x0c;
    HWREG( GPIO_PORTA_BASE + GPIO_O_DIR  ) |= 0x0c;
    HWREG( GPIO_PORTA_BASE + GPIO_O_DATA ) &= ~0x0c;

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void Initialize( void )
// PURPOSE  : Initializes the embedded system
//-----------------------------------------------------------------------------

void Initialize( void )
{
    // Clear all system flags
    g_uiSysFlags = 0;

    // Initialize the motor direction signals
    InitMotorDir();

    // Initialize the on-board blue LED
    InitBlueLED();

    // Initialize the analog-to-digital converter peripheral
    InitADC();

    // Initialize the pulse-width modulation peripheral
    InitPWM();

    // Initialize the system tick
    InitSysTick();

    return;
}

//-----------------------------------------------------------------------------
// FUNCTION : void main( void )
// PURPOSE  : Program entry point
//-----------------------------------------------------------------------------

void main( void )
{
    // Analog-to-Digital Conversion Result (quantized value)
    uint32_t uiADCResult;

    // Initialize the embedded system
    Initialize();

    // main loop
    while( 1 )
    {
        // Sleep (wait for interrupt)
        asm( " wfi" );

        // Check for system tick interval event
        if( g_uiSysFlags & ( 1 << SYSF_TICK ) )
        {
            ControlBlueLED();
            SamplePE3();

            g_uiSysFlags &= ~( 1 << SYSF_TICK );
        }

        // Check for ADC conversion complete event
        if( g_uiSysFlags & ( 1 << SYSF_ADC_DONE ) )
        {
            uiADCResult = ReadADCResult();
            UpdatePWM( uiADCResult );

            g_uiSysFlags &= ~( 1 << SYSF_ADC_DONE );
        }
    }
}

//-----------------------------------------------------------------------------
// END OF MAIN.C
//-----------------------------------------------------------------------------
