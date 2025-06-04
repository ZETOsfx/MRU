#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "FeedbackMotionComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ROBOTSIMULATOR_API UFeedbackMotionComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UFeedbackMotionComponent();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void* SharedPtr = nullptr;
    int32 ShmSize = sizeof(float) * 2;
    FString ShmPath = TEXT("/tmp/UE5_Ctrl");

    FTimerHandle MotionTimerHandle;
    void ReadAndApplyMotion();

    void ApplyMotor(UPhysicsConstraintComponent* Joint, float Speed);
    UPhysicsConstraintComponent* FindJointByName(FName Name);

    UPhysicsConstraintComponent* jointFL = nullptr;
    UPhysicsConstraintComponent* jointFR = nullptr;
    UPhysicsConstraintComponent* jointBL = nullptr;
    UPhysicsConstraintComponent* jointBR = nullptr;
};
