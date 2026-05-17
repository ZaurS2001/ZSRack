#include "plugin.hpp"


Plugin* pluginInstance;


void init(Plugin* p) {
	pluginInstance = p;

	// Add modules here
	p->addModel(modelRand);
	p->addModel(modelZSVCO);
	p->addModel(modelBBOX);
	p->addModel(modelTP_ECHO);
	p->addModel(modelKocmoc);
	p->addModel(modelSTPWTCH);
	p->addModel(modelNoSignal);
	p->addModel(modelMear);
	p->addModel(modeldotflow);
	p->addModel(modelBYT);
	p->addModel(modelLingo);
	p->addModel(modelTAIP);
	p->addModel(modelPACE);
	p->addModel(modelPAUL);
	p->addModel(modelMORSE);
	p->addModel(modelMOUSE);
	p->addModel(modelNUMBS);

	// Any other plugin initialization may go here.
	// As an alternative, consider lazy-loading assets and lookup tables when your module is created to reduce startup times of Rack.
}
