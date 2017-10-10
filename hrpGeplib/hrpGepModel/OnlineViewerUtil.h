#ifndef __HRPGEPMODEL_ONLINEVIEWER_UTIL_H__
#define __HRPGEPMODEL_ONLINEVIEWER_UTIL_H__

#include <hrpGepCorba/OpenHRPCommon.hh>
#include <hrpGepModel/Body.h>
#include <hrpGepModel/World.h>
#include <hrpGepModel/ColdetLinkPair.h>

void setupCharacterPosition(OpenHRP::CharacterPosition &characterPosition, 
                             hrpGep::BodyPtr body); 
void updateCharacterPosition(OpenHRP::CharacterPosition &characterPosition, 
                             hrpGep::BodyPtr body);
void getWorldState(OpenHRP::WorldState& state,  hrpGep::WorldBase& world);
void initWorldState(OpenHRP::WorldState& state, hrpGep::WorldBase& world);
void initWorldState(OpenHRP::WorldState& state, hrpGep::WorldBase& world,
                    std::vector<hrpGep::ColdetLinkPairPtr>& pairs);

#endif
