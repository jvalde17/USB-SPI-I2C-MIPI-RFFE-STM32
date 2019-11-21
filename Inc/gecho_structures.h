//gecho_structures.h
#ifndef GECHOA_STRUCTURES_H

extern struct _settings {
	uint32_t spmi;
	
	uint32_t what_chip;
	uint32_t dynamic;
	uint32_t num_dyn_set;
	uint32_t bitbang_dynamic;
	uint32_t _slot_data_[40][10];
  uint32_t _slot_trigger[40];
	
	uint32_t one_byte[40];
	uint32_t one_byte_addr;
  uint32_t dyn_byte_1[40];
	uint32_t dyn_byte_2[40];
	uint32_t two_byte_addr; 
	
	uint32_t _num_slot;
	uint32_t _slot_delay;
	uint32_t exit_delay;
	int32_t _slot_delay_correction;
	int32_t _slot_delay_correction_2;
	int32_t _tr_gap_delay;
	uint32_t gsid;
	uint32_t pa_usid;
	
	uint32_t gto_dac_now;
	uint32_t gto_dac_prev;
	uint32_t gto_cuda_prior[40];
	
	uint32_t vcore2_dynamic;
	uint32_t vcore2_dynamic_mode;
	uint32_t vcore2_dyn_delay;
	uint32_t vcore2_dyn_delay2;
	uint32_t vcore2_dyn_slot;
	uint32_t vcore2_bh_thread;
	
	uint32_t dvs_vout[4];
	uint32_t dvs_iout[4];
	
} _config;


#endif
