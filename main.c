/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PROT PPU ECC Injection Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
/* LED blink timer period value */
#define LED_BLINK_INTERVAL_MS             (500)

/* User Test Parameters */
#define TEST_PPU_SUFFIX                   (2ul)
#define TEST_PPU_FIXED                    (true)
#define TEST_PPU_MASTER                   (true)

/* Fault Assignment */
#define CY_SYSFAULT_PERI_ECC 24
#define CY_SYSFAULT_PERI_NC_ECC 25

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Data type for 128bit variable */
typedef struct
{
    uint64_t u64[2];
} Uint128Type;

/* Data type for PPU all attributes */
typedef struct
{
    uint32_t att[CY_PROT_ATT_REGS_MAX];
} PpuAllAttributeType;

/* Data type for PPU attribute pointers */
typedef struct
{
    volatile uint32_t* pu32[CY_PROT_ATT_REGS_MAX];
} PpuAttributePointerType;

static const PpuAllAttributeType TEST_ATTRIBUTE =
{
    /*     {       ATT0,       ATT1,       ATT2,       ATT3, }*/
    .att = { 0x1F1F1F1F, 0x1F1F1F1F, 0x00000000, 0x00000000, },
};

/*******************************************************************************
* Function Name: get_Target_Att
********************************************************************************
* Summary:
* This function returns a structure which contains pointers to PPU ATT registers
* which are indicated by input parameters.
*
* Parameters:
*  suffix - PPU ATT register suffix.
*  isFixed - input true for fixed PPU, false for programmable PPU
*  isMaster - input true for master PPU, false for slave PPU
*
* Return:
*  PpuAttributePointerType
*
*******************************************************************************/
static PpuAttributePointerType get_Target_Att(uint32_t suffix, bool isFixed, bool isMaster)
{
    PpuAttributePointerType att = {NULL};
    if (isFixed == true)
    {
        if (isMaster == true)
        {
            att.pu32[0] = &PERI_MS->PPU_FX[suffix].MS_ATT0;
            att.pu32[1] = &PERI_MS->PPU_FX[suffix].MS_ATT1;
            att.pu32[2] = &PERI_MS->PPU_FX[suffix].MS_ATT2;
            att.pu32[3] = &PERI_MS->PPU_FX[suffix].MS_ATT3;
        }
        else
        {
            att.pu32[0] = &PERI_MS->PPU_FX[suffix].SL_ATT0;
            att.pu32[1] = &PERI_MS->PPU_FX[suffix].SL_ATT1;
            att.pu32[2] = &PERI_MS->PPU_FX[suffix].SL_ATT2;
            att.pu32[3] = &PERI_MS->PPU_FX[suffix].SL_ATT3;
        }
    }
    else
    {
        if (isMaster == true)
        {
            att.pu32[0] = &PERI_MS->PPU_PR[suffix].MS_ATT0;
            att.pu32[1] = &PERI_MS->PPU_PR[suffix].MS_ATT1;
            att.pu32[2] = &PERI_MS->PPU_PR[suffix].MS_ATT2;
            att.pu32[3] = &PERI_MS->PPU_PR[suffix].MS_ATT3;
        }
        else
        {
            att.pu32[0] = &PERI_MS->PPU_PR[suffix].SL_ATT0;
            att.pu32[1] = &PERI_MS->PPU_PR[suffix].SL_ATT1;
            att.pu32[2] = &PERI_MS->PPU_PR[suffix].SL_ATT2;
            att.pu32[3] = &PERI_MS->PPU_PR[suffix].SL_ATT3;
        }
    }

    return att;
}

/*******************************************************************************
* Function Name: calculate_Ppu_WordAddr
********************************************************************************
* Summary:
*  This function calculate a WORD_ADDR for the ECC injection
*  E.g. To get the WORD_ADDR for PERI_MS_PPU_FX2_MS_ATT[0:3] then, 
*  input suffix = 2,  isFixed = true,  isMaster = true
*  The ADDR can be calculated as follows.
*   PERI_MS_PPU_PRx_SL_ATT0-3: x * 2
*   PERI_MS_PPU_PRx_MS_ATT0-3: x * 2 + 1
*   PERI_MS_PPU_FXx_SL_ATT0-3: x * 2 + 64
*   PERI_MS_PPU_FXx_MS_ATT0-3: x * 2 + 65
*
* Parameters:
*  suffix - PPU ATT register suffix.
*  isFixed - input true for fixed PPU, false for programmable PPU
*  isMaster - input true for master PPU, false for slave PPU
*
* Return:
*  calculated PPU word address
*
*******************************************************************************/
static uint32_t calculate_Ppu_WordAddr(uint32_t suffix, bool isFixed, bool isMaster)
{
    uint32_t offset = 0ul;
    offset += (isFixed  ? 64 : 0);
    offset += (isMaster ? 1  : 0);

    return (suffix * 2ul) + offset;
}

/*******************************************************************************
* Function Name: make_ActualWord_For_Ppu_Ecc
********************************************************************************
* Summary:
*  This genrate actual word used for parity calculation
*
* Parameters:
*  attValue - target PPU ATT value for ECC error injection
*
* Return:
*  Uint128Type - actual word
*
*******************************************************************************/
static Uint128Type make_ActualWord_For_Ppu_Ecc(const PpuAllAttributeType* attValue)
{
    /* Make PC[1] ~ PC[15] */
    uint8_t pc[16];
    for(uint32_t pcIdx = 1; pcIdx < 16; pcIdx += 1)
    {
        uint32_t bitPos = (pcIdx*8) % 32;
        uint32_t attSuffix = pcIdx / 4;
        pc[pcIdx] = (attValue->att[attSuffix] >> bitPos) & 0x1F;
    }

    /* Make ACTUAL_WORD form the PC0~PC15 */
    /* Note PC[0] is not used */
    Uint128Type acturalWord = {0ul};
    acturalWord.u64[0] =  (uint64_t)pc[1]           |
                         ((uint64_t)pc[2]  << 5ul)  |
                         ((uint64_t)pc[3]  << 10ul) |
                         ((uint64_t)pc[4]  << 15ul) |
                         ((uint64_t)pc[5]  << 20ul) |
                         ((uint64_t)pc[6]  << 25ul) |
                         ((uint64_t)pc[7]  << 30ul) |
                         ((uint64_t)pc[8]  << 35ul) |
                         ((uint64_t)pc[9]  << 40ul) |
                         ((uint64_t)pc[10] << 45ul) |
                         ((uint64_t)pc[11] << 50ul) |
                         ((uint64_t)pc[12] << 55ul) |
                         ((uint64_t)pc[13] << 60ul);
    acturalWord.u64[1] = ((uint64_t)pc[13] >> 4ul) |
                         ((uint64_t)pc[14] << 1ul) |
                         ((uint64_t)pc[15] << 6ul);

    return acturalWord;
}

/*******************************************************************************
* Function Name: make_CodeWord_For_Ppu_Ecc
********************************************************************************
* Summary:
*  This genrate code word used for parity calculation
*
* Parameters:
*  attValue - target PPU ATT value for ECC error injection
*  ppuWordAddr - target address
*
* Return:
*  Uint128Type - code word
*
*******************************************************************************/
static Uint128Type make_CodeWord_For_Ppu_Ecc(const PpuAllAttributeType* attValue, uint32_t ppuWordAddr)
{
    /* CODEWORD_SW [74:0] = ACTUALWORD [74:0]; */
    /* Other bits = 0 */
    Uint128Type word = make_ActualWord_For_Ppu_Ecc(attValue);

    /* CODEWORD_SW [75] = 0 (wounding bit); */
    /* nothing to do */

    /* CODEWORD_SW [86: 76] = ADDR [10:0]; */
    word.u64[1] |= (ppuWordAddr & 0x000003FFul) << (76ul - 64ul);

    return word;
}

/*******************************************************************************
* Function Name: do_AND_128bit
********************************************************************************
* Summary:
*  This do logical AND operation on two 128bit values
*
* Parameters:
*  a - 1st value
*  b - 2nd value
*
* Return:
*  Uint128Type - code word
*
*******************************************************************************/
static Uint128Type do_AND_128bit(Uint128Type a, Uint128Type b)
{
    Uint128Type result;
    result.u64[0] = a.u64[0] & b.u64[0];
    result.u64[1] = a.u64[1] & b.u64[1];
    return result;
}

/*******************************************************************************
* Function Name: do_Reduction_XOR_128bit
********************************************************************************
* Summary:
*  This do reduction XOR operation on specified 128bit value and get the parity
*
* Parameters:
*  data - target value
*
* Return:
*  uint8_t - parity
*
*******************************************************************************/
static uint8_t do_Reduction_XOR_128bit(Uint128Type data)
{
    uint64_t parity = 0;
    uint64_t bit    = 0;
    for(uint64_t iPos = 0; iPos < 64; iPos++)
    {
        bit = (data.u64[0] & (1ull << iPos)) >> iPos;
        parity ^= bit;
    }

    for(uint64_t iPos = 0; iPos < 64; iPos++)
    {
        bit = (data.u64[1] & (1ull << iPos)) >> iPos;
        parity ^= bit;
    }

    return (uint8_t)parity;
}

/*******************************************************************************
* Function Name: generate_Parity
********************************************************************************
* Summary:
*  This calculate parity for specified 128bit value
*
* Parameters:
*  word - target value
*
* Return:
*  uint8_t - parity
*
*******************************************************************************/
static uint8_t generate_Parity(Uint128Type word)
{
    static const Uint128Type ECC_P[8] =
    {
        {{0x44844a88952aad5bull, 0x01bfbb75be3a72dcull}},
        {{0x1108931126b3366dull, 0x02df76f9dd99b971ull}},
        {{0x06111c2238c3c78eull, 0x04efcf9f9ad5ce97ull}},
        {{0x9821e043c0fc07f0ull, 0x08f7ecf6ed674e6cull}},
        {{0xe03e007c00fff800ull, 0x10fb7baf6ba6b5a6ull}},
        {{0xffc0007fff000000ull, 0x20fdb7cef36cab5bull}},
        {{0xffffff8000000000ull, 0x40fedd7b74db55abull}},
        {{0xd44225844ba65cb7ull, 0x807f000007ffffffull}},
    };

    uint8_t ecc = 0;
    for (uint32_t cnt = 0; cnt < (sizeof(ECC_P) / sizeof(ECC_P[0])); cnt++)
    {
        ecc |= (do_Reduction_XOR_128bit(do_AND_128bit(word, ECC_P[cnt])) << cnt);
    }

    return ecc;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    Cy_SCB_UART_Init(UART_HW, &UART_config, NULL);
    Cy_SCB_UART_Enable(UART_HW);
    cy_retarget_io_init(UART_HW);

    /* Initialize the User LED */
    result = Cy_GPIO_Pin_Init(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, &CYBSP_USER_LED_config);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PPU ECC Error Injection example "
           "****************** \r\n\n");

    /* Configure Fault report */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, (cy_en_SysFault_source_t)CY_SYSFAULT_PERI_ECC);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, (cy_en_SysFault_source_t)CY_SYSFAULT_PERI_NC_ECC);

    /* Set user defined ATT value to the target registers */
    volatile PpuAttributePointerType att = get_Target_Att(TEST_PPU_SUFFIX, TEST_PPU_FIXED, TEST_PPU_MASTER);
    *att.pu32[0] = TEST_ATTRIBUTE.att[0];
    *att.pu32[1] = TEST_ATTRIBUTE.att[1];
    *att.pu32[2] = TEST_ATTRIBUTE.att[2];
    *att.pu32[3] = TEST_ATTRIBUTE.att[3];

    volatile uint32_t eccCtl = PERI->ECC_CTL;

    /* Calculate ADDR[10:0] and set to PERI_ECC_CTL.WORD_ADDR. */
    uint32_t wordAddr = calculate_Ppu_WordAddr(TEST_PPU_SUFFIX, TEST_PPU_FIXED, TEST_PPU_MASTER);
    eccCtl |= (wordAddr & (PERI_ECC_CTL_WORD_ADDR_Msk >> PERI_ECC_CTL_WORD_ADDR_Pos) << PERI_ECC_CTL_WORD_ADDR_Pos);

    /* Generate the ECC parity and set to PERI_ECC_CTL.PARITY. */
    Uint128Type word = make_CodeWord_For_Ppu_Ecc(&TEST_ATTRIBUTE ,wordAddr);
    uint8_t parity = generate_Parity(word);
    eccCtl |= ((parity & (PERI_ECC_CTL_PARITY_Msk >> PERI_ECC_CTL_PARITY_Pos)) << PERI_ECC_CTL_PARITY_Pos);

    /* Set the PERI_ECC_CTL.ECC_INJ_EN to "1". */
    eccCtl |= (1 << PERI_ECC_CTL_ECC_INJ_EN_Pos);

    PERI->ECC_CTL = eccCtl;

    /*********************************/
    /* Test 1: Reading without error */
    /*********************************/
    printf("**Test 1: Reading without error**\r\n");

    /* Read the target PPU structure ATT0-3. */
    (void)*att.pu32[0];
    (void)*att.pu32[1];
    (void)*att.pu32[2];
    (void)*att.pu32[3];

    /* Read the fault error source */
    cy_en_SysFault_source_t     faultSource;
    faultSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);

    /* Expectation: no fault */
    if (faultSource == CY_SYSFAULT_NO_FAULT)
    {
        printf("-->success\r\n");
    }
    else
    {
        printf("-->failure: unexpected result(0x%" PRIu32 ")\r\n", (uint32_t)faultSource);
        CY_ASSERT(0);
    }

    /* Clear the fault status */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);

    /********************************************************/
    /* Test 2: Reading with 1-bit error (correctable error) */
    /********************************************************/
    printf("**Test 2: Reading with 1-bit correctable error**\r\n");

    /* Set attribute value with 1-bit being inverted (Correctable error). */
    *att.pu32[0] = TEST_ATTRIBUTE.att[0] ^ 0x00001000ul;

    /* Write the correct value. Note that because the ATT has 128 bits,
       only 32-bits writing initiates read-modify-write.
       Thus, this instruction contains reading which will cause correctable ECC error. */
    *att.pu32[0] = TEST_ATTRIBUTE.att[0];

    /* Read the fault error source */
    faultSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);

    /* Expectation: correctable ECC error */
    if (faultSource == (cy_en_SysFault_source_t)CY_SYSFAULT_PERI_ECC)
    {
        printf("-->success\r\n");
    }
    else
    {
        printf("-->failure: unexpected result(0x%" PRIu32 ")\r\n", (uint32_t)faultSource);
        CY_ASSERT(0);
    }

    /* Clear the fault status */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);

    /*************************************************************/
    /* Test 3: Reading with 2-bits error (non-correctable error) */
    /*************************************************************/
    printf("**Test 3: Reading with 2-bit non-correctable error**\r\n");

    /* Set attribute value with 2-bits being inverted (Non-correctable error). */
    *att.pu32[0] = TEST_ATTRIBUTE.att[0] ^ 0x00101000ul;

    /* Write the correct value. Note that because the ATT has 128 bits,
       only 32-bits writing initiates read-modify-write.
       Thus, this instruction contains reading which will cause non-correctable ECC error. */
    *att.pu32[0] = TEST_ATTRIBUTE.att[0];

    /* Read the fault error source */
    faultSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);

    /* Expectation: non-correctable ECC error */
    if (faultSource == (cy_en_SysFault_source_t)CY_SYSFAULT_PERI_NC_ECC)
    {
        printf("-->success\r\n");
    }
    else
    {
        printf("-->failure: unexpected result(0x%" PRIu32 ")\r\n", (uint32_t)faultSource);
        CY_ASSERT(0);
    }

    /* Clear the fault status */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);

    for (;;)
    {
        Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        Cy_SysLib_Delay(LED_BLINK_INTERVAL_MS);
    }
}

/* [] END OF FILE */
