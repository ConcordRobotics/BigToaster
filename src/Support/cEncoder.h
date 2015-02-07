#ifndef CENCODER_H
#define CENCODER_H

#include "WPILib.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class cEncoder : public PIDSource {
protected:
	~cEncoder() {};
private:
	int mode;
public:
	Encoder* encoder;
	double PIDGet();
	cEncoder(Encoder* iEncoder, unsigned int iMode);
};

#endif
