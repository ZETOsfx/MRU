#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "SharedMemoryCaptureComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ROBOTSIMULATOR_API USharedMemoryCaptureComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    USharedMemoryCaptureComponent();

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
    USceneCaptureComponent2D* RGBComponent = nullptr;
    USceneCaptureComponent2D* DepthComponent = nullptr;

    void* SharedMemoryPtrRGB = nullptr;
    void* SharedMemoryPtrDepth = nullptr;

    int32 Width = 1280;
    int32 Height = 720;
    int32 SizeRGB = 1280 * 720 * 4;        // 4 bytes per pixel (RGBA8)
    int32 SizeDepth = 1280 * 720 * 4;      // 1 float per pixel (4 bytes)

    FString SharedPathRGB = "/tmp/UE5_SharedMemory_RGB";
    FString SharedPathDepth = "/tmp/UE5_SharedMemory_Depth";

    float TimeSinceLastWrite = 0.f;

    void InitSharedMemory(const FString& Path, int32 Size, void** OutPtr);
    void WriteToSharedMemory(void* Ptr, const void* Data, int32 Size);
};
