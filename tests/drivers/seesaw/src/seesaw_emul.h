#ifndef SEESAW_EMUL_H_
#define SEESAW_EMUL_H_

#define SEESAW_REGISTER_COUNT 15

int seesaw_mock_set_register(void *data_ptr, int reg, uint32_t value);
int seesaw_mock_get_register(void *data_ptr, int reg, uint32_t *value_ptr);

#endif /* SEESAW_EMUL_H_ */