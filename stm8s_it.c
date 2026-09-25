 #include "stm8s.h"


 extern void TimingDelay_Decrement(void);


  INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {
  TimingDelay_Decrement();

  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
 }

