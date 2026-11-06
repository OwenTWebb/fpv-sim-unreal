#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "FpvGameMode.generated.h"

/** Builds the sky and lights, the procedural world and the quad, and views through the quad's camera. */
UCLASS()
class FPVSIM_API AFpvGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AFpvGameMode();
	virtual void BeginPlay() override;
};
