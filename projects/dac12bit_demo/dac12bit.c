/*
 * File:		dac12bit.c
 * Purpose:		function implementations
 *
 */

#include "dac12bit.h"
#include <stdint.h>



   byte DAC1_UP_PTR =0;

   byte watermarkIndicator[16][16]; 
   int DACC2_DACBFUP= 0 ; 
   int DACC2_DACBFRP = 0 ; 
  
    
    int bottom_isr = 0 ;
    int top_isr = 0 ;
    int watermark_isr = 0 ;
    
    
 /********************************************************************/
void dac0_12bit_isr (void){
  
  
if ((DAC_SR_REG(DAC0_BASE_PTR)&DAC_SR_DACBFRPBF_MASK) ) {
 //Buffer read pointer is equal to top limit 
  bottom_isr++;
   DAC_SR_REG(DAC0_BASE_PTR)=Clear_DACBFRPBF;
}
else if ((DAC_SR_REG(DAC0_BASE_PTR)&DAC_SR_DACBFRPTF_MASK) ){
//buffer read pointer is zero
  top_isr++;
    DAC_SR_REG(DAC0_BASE_PTR)=Clear_DACBFRPTF ;
}
else if ((DAC_SR_REG(DAC0_BASE_PTR)&DAC_SR_DACBFWMF_MASK)){
  //buffer read pointer reaches the watermark level
  watermark_isr++;
DAC_SR_REG(DAC0_BASE_PTR)= Clear_DACBFWMF ;
}


return;
}


void dac1_12bit_isr (void){  
  
  
 //Record current read pointer position and up limit position      
DACC2_DACBFUP =  DAC_C2_DACBFUP_MASK & DAC_C2_REG(DAC1_BASE_PTR);
DACC2_DACBFRP =  (DAC_C2_DACBFRP_MASK & DAC_C2_REG(DAC1_BASE_PTR))>>4;  
           
 //Record the watermark flag at each read pointer position: Watermark conditions marked as 0x04 else 0x00.
 watermarkIndicator[DACC2_DACBFUP][DACC2_DACBFRP]= DAC_SR_REG(DAC1_BASE_PTR)& DAC_SR_DACBFWMF_MASK  ; 
  
if ((DAC_SR_REG(DAC1_BASE_PTR)&DAC_SR_DACBFRPBF_MASK) ) {
 //Buffer read pointer is equal to top limit 
  bottom_isr++;
  
    DAC1_UP_PTR++;
    if (DAC1_UP_PTR !=16){
     //set up limit base on i counter
   DAC_C2_REG(DAC1_BASE_PTR)= DAC_SET_PTR_AT_BF(0) | DAC_SET_PTR_UP_LIMIT(DAC1_UP_PTR); 
    }
  DAC_SR_REG(DAC1_BASE_PTR)=Clear_DACBFRPBF;

}
else if ((DAC_SR_REG(DAC1_BASE_PTR)&DAC_SR_DACBFRPTF_MASK) ){
//buffer read pointer is zero
  top_isr++;
  DAC_SR_REG(DAC1_BASE_PTR)=Clear_DACBFRPTF ;
  
}
else if ((DAC_SR_REG(DAC1_BASE_PTR)&DAC_SR_DACBFWMF_MASK)){
  //buffer read pointer reaches the watermark level
  watermark_isr++;
DAC_SR_REG(DAC1_BASE_PTR)= Clear_DACBFWMF ;
}
  
 //DAC_SR_REG(DAC1_BASE_PTR)=0x00;//clear all flags

return ; 
}    
   


/**************************************************************/ 
    
    
    
 /* dac0_enable only enable the clock to DAC0. WIll need this before */
void dac0_clk_enable (void ){
SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK ; //Allow clock to enable DAC0


}

void dac1_clk_enable(void){
SIM_SCGC2 |= SIM_SCGC2_DAC1_MASK ; //Allow clock to enable DAC1
}
void dac0_1_clk_enable(void){
dac0_clk_enable();
dac1_clk_enable();

}
void dacx_init(void){


DAC0_C0 |= DAC_C0_DACEN_MASK ; 


}
               
               
               
void errorhandler (void){
  while(1){ /* loop forever*/  };

}

void VREF_Init(void){

SIM_SCGC4 |= SIM_SCGC4_VREF_MASK ;
    VREF_SC = 0x81 ;// Enable Vrefo and select internal mode
    //VREF_SC = 0x82; // Tight-regulation mode buffer enabled is reconmended over low buffered mode
    while (!(VREF_SC & VREF_SC_VREFST_MASK)  ){} // wait till the VREFSC is stable

}

void DAC12_Vin_SWtrig(DAC_MemMapPtr dacx_base_ptr, unsigned char VrefSel)  {

     DAC_C0_REG(dacx_base_ptr)  = (
     DAC_BFB_PTR_INT_DISABLE |  // DACBIE =0 , buffer read pointer  bottom flag interrupt is disabled             
     DAC_BFT_PTR_INT_DISABLE  | // DACTIE = 0 ,buffer read pointer top flag interrupt is disabled            
     DAC_BFWM_INT_DISABLE |     // DACWIE = 0, buffer water mark interrupt disabled            
     DAC_HP_MODE    |           // LPEN = 0, low power mode            
     DAC_SW_TRIG_STOP |         // NO Software trigger yet            
     DAC_SEL_SW_TRIG |          // DACTSEL =1, Databuffer outputs to DACO pin whenever a write to DACDAT0     
     VrefSel |          
     DAC_ENABLE                 //DAC enalbed
     );  

    if ( VrefSel == DAC_SEL_VREFO ) {
    VREF_Init();
    }// end of if

     DAC_C1_REG(dacx_base_ptr)= ( 
     DAC_BF_DISABLE  |  // Buffer read pointer is disabled, converted data is always the first word of buffer = DACCCDAT0            
     DAC_BF_NORMAL_MODE |          
     DAC_BFWM_1WORD |
     DAC_DMA_DISABLE  
     ) ;

DAC_C2_REG(dacx_base_ptr) = DAC_SET_PTR_AT_BF(0)| DAC_SET_PTR_UP_LIMIT(0x0f);
}// end of DAC12_Vin_SWtrig

 void DAC12_VreferenceInit(DAC_MemMapPtr dacx_base_ptr,unsigned char Vinselect){
 

      
    DAC12_Vin_SWtrig(dacx_base_ptr, Vinselect); // input 1 to function to select  Vin = Vext = VDDA  
    SET_DACx_BUFFER(dacx_base_ptr, 0,0); //Intialize Buffer 0  to value 0.
    

 }//end of DAC12_VreferenceTest

void DAC12_VreferenceRamp(void){

 int j = 0 ;
    int increment = 0 ; 
    while(1){ 
    for (increment = 0 ; increment <4096; increment++) {
     
      //Set value for DACx output. This function assume dacx is configured to non-buffer mode, 
      //The analog DACx output is reflected upon the change of value in buffer0 , not the read pointer.
     SET_DACx_BUFFER(DAC0_BASE_PTR, 0,increment);  
     SET_DACx_BUFFER(DAC1_BASE_PTR,0,increment);  
    
      for (j=0;j<1000;j++){}  // random delay
     }//end of outer for loop
  
    }//end of while loop
}
void DAC12_buffered (DAC_MemMapPtr dacx_base_ptr, byte WatermarkMode, byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos,byte BuffUpLimit){

    DAC_C0_REG(dacx_base_ptr) = (
    DAC_BFB_PTR_INT_DISABLE |             
    DAC_BFT_PTR_INT_DISABLE |             
    DAC_BFWM_INT_DISABLE  |             
    DAC_HP_MODE    |             
    DAC_SW_TRIG_STOP |            
    TrigMode |  
    Vreference |     
    DAC_ENABLE    // DAC enalbed
     );  
    
    if ( Vreference == DAC_SEL_VREFO ) {
    VREF_Init();
    }// end of if


    DAC_C1_REG(dacx_base_ptr) = ( 
    DAC_BF_ENABLE  |  //Buffer Enabled            
    WatermarkMode |  // set 1, 2, 3,or 4 word(s) mode           
    BuffMode        // set traversal mode, normal, swing, or onetime   
    ) ;

    DAC_C2_REG(dacx_base_ptr) = BuffInitPos | BuffUpLimit;

}//end of DAC12_buffered 


void DAC12_Buff_Init_Plus256( DAC_MemMapPtr dacx_base_ptr){
  
     int data = 0; 
     byte i =  0 ;
     //Uncomment the follows to test for buffer mode
     data = 0 ;  
     // for loop: Initializes the buffer words so that next buffer has an increment of 256 except last one (255)  
     for (i=0 ;i < 16 ; i++){
      
      if(i == 15){//Last buffer.The last word buffer (16th word) will have the value (16*0xff)-1 = 4096-1=4095=0xFFF. 
       data+= 255; 
      }
      else{ //Not last buffer.The next word buffer will have increment of 0xFF from previous word buffer.         
       data+= 256;             
      }  //end of if-else statement
      
      SET_DACx_BUFFER( dacx_base_ptr, i, data);
     
     }// end of for loop
 }//end of DAC12_Buff_Init_Plus256

void DAC12_SoftwareTriggerLoop(void){
int j = 0 ;

while (1){
       
      DAC_C0_REG(DAC0_BASE_PTR) |= DAC_SW_TRIG_NEXT; //SW trigger to advance to DAC0's next word buffer
      DAC_C0_REG(DAC1_BASE_PTR) |= DAC_SW_TRIG_NEXT; //SW trigger to advance to DAC1's next word buffer
      for (j=0;j<10000;j++){}  // random delay
        
     }


}


 void DAC12_SoftTrigBuffInit(DAC_MemMapPtr dacx_base_ptr,byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit)
 { 
  
     
    //Initilized DAC12  
    DAC12_buffered( dacx_base_ptr,DAC_BFWM_1WORD, BuffMode, Vreference, TrigMode,BuffInitPos, BuffUpLimit) ;
    DAC12_Buff_Init_Plus256(dacx_base_ptr);//init buffer to with 256 increment with following values word 0(=256), Word 1 (=256+256) .... to word 15 (=4096)
   
 } //end of DAC12_SoftTrigBuff
 
 
 int SET_DACx_BUFFER( DAC_MemMapPtr dacx_base_ptr, byte dacindex, int buffval){
   int temp = 0 ; 
   // initialize all 16 words buffer with a the value buffval
   DAC_DATL_REG(dacx_base_ptr, dacindex)  =   (buffval&0x0ff); 
   DAC_DATH_REG(dacx_base_ptr, dacindex)  =   (buffval&0xf00) >>8;                                
   temp =( DAC_DATL_REG(dacx_base_ptr, dacindex)|( DAC_DATH_REG(dacx_base_ptr, dacindex)<<8)); 
    //if(temp != buffval){ while(1){} }
  return temp ; 
 }// end of SET_DACx_BUFFER


void DAC12_HWTrigBuff(DAC_MemMapPtr dacx_base_ptr, byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit) {
    DAC12_buffered( dacx_base_ptr,0, BuffMode, Vreference, TrigMode,BuffInitPos, BuffUpLimit) ;
    DAC12_Buff_Init_Plus256(dacx_base_ptr);//init buffer to with 256 increment with following values word 0(=256), Word 1 (=256+256) .... to word 15 (=4096)

   
   //Initialize PDB for DAC hardware trigger
   PDB_DAC0_TriggerInit();  
   PDB_DAC1_TriggerInit();  

} //end of DAC12_HWTrigBuff


void PDB_DAC0_TriggerInit(void) {
    SIM_SCGC6 |= SIM_SCGC6_PDB_MASK ; // enable system clock to PDB
    PDB0_SC = PDB_SC_PDBEN_MASK ; // enable PDB module
    PDB0_DACINT0 = 60000 ;// Effective after writting PDBSC_DACTOE = 1, DAC output changes are base on the interval defined by this value
    PDB0_SC |=PDB_SC_LDOK_MASK    ;// load the value assigned to PDB_DACINT0 to register   
    PDB0_SC |= PDB_SC_TRGSEL(15); // software trigger
    PDB0_SC |=PDB_SC_CONT_MASK ;// CONT mode
    PDB0_DACINTC0 |= PDB_INTC_TOE_MASK ;// DAC output delay from PDB Software trigger
    PDB0_SC |= PDB_SC_SWTRIG_MASK ;// reset counter to 0. This triggers the PDB input. 

  } //end of PDB_DAC_TriggerInit
void PDB_DAC1_TriggerInit(void) {
    SIM_SCGC6 |= SIM_SCGC6_PDB_MASK ; // enable system clock to PDB
    PDB0_SC = PDB_SC_PDBEN_MASK ; // enable PDB module
    PDB0_DACINT1 = 60000 ;// Effective after writting PDBSC_DACTOE = 1, DAC output changes are base on the interval defined by this value
    PDB0_SC |=PDB_SC_LDOK_MASK    ;// load the value assigned to PDB_DACINT1  to register   
    PDB0_SC |= PDB_SC_TRGSEL(15); // software trigger
    PDB0_SC |=PDB_SC_CONT_MASK ;// CONT mode
    PDB0_DACINTC1 |= PDB_INTC_TOE_MASK ;// DAC output delay from PDB Software trigger
    PDB0_SC |= PDB_SC_SWTRIG_MASK ;// reset counter to 0. This triggers the PDB input. 
  } //end of PDB_DAC_TriggerInit

void DACx_reset_dac0_1_reg_values(void){
DACx_register_reset_por_values (DAC0_BASE_PTR);
DACx_register_reset_por_values (DAC1_BASE_PTR);
}//end of DACx_reset_dac0_1_reg_values


void DACx_register_reset_por_values (DAC_MemMapPtr dacx_base_ptr){
  unsigned char dacbuff_index = 0 ;   

for (dacbuff_index=0; dacbuff_index<16;dacbuff_index++){
    SET_DACx_BUFFER( dacx_base_ptr, dacbuff_index, DACx_DAT_RESET); 
  }
   
     DAC_SR_REG(dacx_base_ptr) = DACx_SR_RESET ;
     DAC_C0_REG(dacx_base_ptr) = DACx_C0_RESET ;
     DAC_C1_REG(dacx_base_ptr) = DACx_C1_RESET;
     DAC_C2_REG(dacx_base_ptr) = DACx_C2_RESET;
} //end of DACx_register_reset_por_values



void DAC12_Interrupt_Init(DAC_MemMapPtr dacx_base_ptr,byte watermark, byte zerobuffer, byte uplimitbuffer ){

  
  if (dacx_base_ptr == DAC0_BASE_PTR ) {
     NVICICPR2 |= (1<<17);
     NVICISER2 |= (1<<17);
  }else {
     NVICICPR2 |= (1<<18);
     NVICISER2 |= (1<<18);
  }
 
  DAC_C0_REG(dacx_base_ptr) |= watermark |zerobuffer | uplimitbuffer ; 
}

void DAC12_WatermarkBuffInterrupt(DAC_MemMapPtr dacx_base_ptr, byte WatermarkBuffMode ,byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit) {
 
    DAC12_buffered(dacx_base_ptr,WatermarkBuffMode, BuffMode, Vreference, TrigMode,BuffInitPos, BuffUpLimit) ;
    DAC_SR_REG(dacx_base_ptr) = 0; //clear all the flags  
    
     DAC12_Interrupt_Init(dacx_base_ptr, DAC_BFWM_INT_ENABLE, DAC_BFT_PTR_INT_ENABLE,DAC_BFB_PTR_INT_ENABLE);
     EnableInterrupts ; 
   
       return ; 
}






 






















