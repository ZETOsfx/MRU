#include "SharedMemoryCaptureComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

USharedMemoryCaptureComponent::USharedMemoryCaptureComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void USharedMemoryCaptureComponent::BeginPlay()
{
    Super::BeginPlay();

    // Поиск камер по имени
    AActor* Owner = GetOwner();
    TArray<USceneCaptureComponent2D*> Components;
    Owner->GetComponents<USceneCaptureComponent2D>(Components);

    for (USceneCaptureComponent2D* Comp : Components)
    {
        if (Comp->GetName().Contains("RGB"))
            RGBComponent = Comp;
        else if (Comp->GetName().Contains("Depth"))
            DepthComponent = Comp;
    }

    if (!RGBComponent || !DepthComponent)
    {
        UE_LOG(LogTemp, Error, TEXT("Камеры не найдены!"));
        return;
    }

    // Shared memory
    InitSharedMemory(SharedPathRGB, SizeRGB, &SharedMemoryPtrRGB);
    InitSharedMemory(SharedPathDepth, SizeDepth, &SharedMemoryPtrDepth);
}

void USharedMemoryCaptureComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (SharedMemoryPtrRGB) munmap(SharedMemoryPtrRGB, SizeRGB);
    if (SharedMemoryPtrDepth) munmap(SharedMemoryPtrDepth, SizeDepth);
    Super::EndPlay(EndPlayReason);
}

void USharedMemoryCaptureComponent::InitSharedMemory(const FString& Path, int32 Size, void** OutPtr)
{
    int fd = open(TCHAR_TO_ANSI(*Path), O_CREAT | O_RDWR, 0666);
    if (fd == -1 || ftruncate(fd, Size) == -1)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to open or resize shared memory: %s"), *Path);
        if (fd != -1) close(fd);
        return;
    }

    *OutPtr = mmap(NULL, Size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (*OutPtr == MAP_FAILED)
    {
        UE_LOG(LogTemp, Error, TEXT("mmap failed: %s"), *Path);
        *OutPtr = nullptr;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Shared memory ready: %s"), *Path);
    }
}

void USharedMemoryCaptureComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    TimeSinceLastWrite += DeltaTime;
    if (TimeSinceLastWrite < 0.05f) return;
    TimeSinceLastWrite = 0.f;

    if (!RGBComponent || !DepthComponent) return;

    // Принудительный захват кадра (если CaptureEveryFrame выключен)
    RGBComponent->CaptureScene();
    DepthComponent->CaptureScene();

    // Синхронизация с GPU перед чтением
    FlushRenderingCommands();

    // Получить размеры из RenderTarget (на случай если изменились)
    Width = RGBComponent->TextureTarget->SizeX;
    Height = RGBComponent->TextureTarget->SizeY;
    SizeRGB = Width * Height * 4;
    SizeDepth = Width * Height * 4;

    // RGB
    if (RGBComponent->TextureTarget && SharedMemoryPtrRGB)
    {
        TArray<FColor> Pixels;
        RGBComponent->TextureTarget->GameThread_GetRenderTargetResource()->ReadPixels(Pixels);
        if (Pixels.Num() == Width * Height)
            WriteToSharedMemory(SharedMemoryPtrRGB, Pixels.GetData(), SizeRGB);
    }

    // DEPTH (RGB16F)
    if (DepthComponent->TextureTarget && SharedMemoryPtrDepth)
    {
        TArray<FFloat16Color> DepthPixels;
        DepthComponent->TextureTarget->GameThread_GetRenderTargetResource()->ReadFloat16Pixels(DepthPixels);

        if (DepthPixels.Num() == Width * Height)
        {
            TArray<float> DepthData;
            DepthData.Reserve(DepthPixels.Num());

            for (const FFloat16Color& Pixel : DepthPixels)
            {
                DepthData.Add(Pixel.R.GetFloat()); // R
            }

            WriteToSharedMemory(SharedMemoryPtrDepth, DepthData.GetData(), SizeDepth);
        }
    }
}

void USharedMemoryCaptureComponent::WriteToSharedMemory(void* Ptr, const void* Data, int32 Size)
{
    if (Ptr && Data)
        FMemory::Memcpy(Ptr, Data, Size);
}
