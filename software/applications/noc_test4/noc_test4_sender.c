#include "noc_test4.h"

void noc_test4_sender (void)
{
	int32_t i;
	uint32_t crc;
	int8_t buf[500];
	int16_t val, channel;
	
	
	if (hf_comm_create(hf_selfid(), 1000, 0))
		panic(0xff);

	delay_ms(50);
	
	srand(hf_cpuid());
	
	// generate a unique channel number for this CPU
	channel = hf_cpuid();
	//while (1){

		for (i = 0; i < sizeof(buf)-4; i++)
			buf[i] = random() % 255;
			
		crc = hf_crc32(buf, sizeof(buf)-4);
		memcpy(buf+sizeof(buf)-4, &crc, 4);
		val = hf_send(2, 5000, buf, sizeof(buf), channel);
		//val = hf_send(3, 5000, buf, sizeof(buf), channel);
		//val = hf_send(4, 5000, buf, sizeof(buf), channel);
		//val = hf_send(5, 5000, buf, sizeof(buf), channel);
		//val = hf_send(6, 5000, buf, sizeof(buf), channel);

		if (val)
			printf("hf_send(): error %d\n", val);
		else
			printf("hf_send(): channel=%d\n", channel);
			
		delay_ms(10);
			
	//}
}