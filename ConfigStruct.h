//============= BATTERY INFO  =====
struct config_t {
	uint32_t nCells;
	uint32_t AH;
	uint32_t CV; // per cell
	uint32_t CC; // max output current
	uint32_t mainsC; // max input current
	// sensor config
	float Vcal;
	float Vcal_k;
	float mVcal;
	float Ccal;
};