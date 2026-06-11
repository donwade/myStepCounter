extern void set_factoryDefaults(void);
uint16_t RMWFeature( uint16_t enumReg, uint8_t LHS, uint8_t RHS, uint16_t value, char *msg = NULL);
uint32_t getStepsTaken();
uint8_t getActivity();
const char* activity2string(uint8_t act);
void resetStepCtr(void);

