#ifndef __SYS_IRQ_H__
#define __SYS_IRQ_H__

#ifdef __cplusplus
extern "C"
{
#endif

extern void sys_irq_shell();
extern void sys_irq_timer_10ms();
extern void sys_irq_timer_5s();

#ifdef __cplusplus
}
#endif

#endif // __SYS_IRQ_H__
