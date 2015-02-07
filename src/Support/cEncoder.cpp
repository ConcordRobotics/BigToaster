
#include <Support/cEncoder.h>


cEncoder::cEncoder(Encoder* iEncoder, unsigned int iMode)  {
	mode = iMode;
	encoder = iEncoder;
}

double cEncoder::PIDGet() {
	if (mode == PIDSourceParameter::kDistance) {
		return encoder->GetDistance();
	} else if (mode == PIDSourceParameter::kRate) {
		return encoder->GetRate();
	} else return 0.0;
}
