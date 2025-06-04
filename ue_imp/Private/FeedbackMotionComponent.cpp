#include "FeedbackMotionComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// Настройки максимальных значений
#define MAX_FORWARD_SPEED 20.0f // см/с
#define MAX_TURN_SPEED    20.0f // град/с
#define TORQUE_FACTOR_FORWARD 120.0f
#define TORQUE_FACTOR_TURN    200.0f

UFeedbackMotionComponent::UFeedbackMotionComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UFeedbackMotionComponent::BeginPlay()
{
    Super::BeginPlay();

    // Попытка открыть Shared Memory, если не существует — создаём
    int fd = open(TCHAR_TO_ANSI(*ShmPath), O_CREAT | O_RDWR, 0666);
    if (fd == -1)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to open/create shared memory: %s"), *ShmPath);
        return;
    }

    ftruncate(fd, ShmSize); // Резервируем размер
    SharedPtr = mmap(nullptr, ShmSize, PROT_READ, MAP_SHARED, fd, 0);
    ftruncate(fd, ShmSize);
    close(fd);

    if (!SharedPtr || SharedPtr == MAP_FAILED)
    {
        UE_LOG(LogTemp, Warning, TEXT("Failed to map shared memory"));
        SharedPtr = nullptr;
        return;
    }

    // Поиск физ. соединений колес
    jointFR = FindJointByName(TEXT("jointFR"));
    jointFL = FindJointByName(TEXT("jointFL"));
    jointBR = FindJointByName(TEXT("jointBR"));
    jointBL = FindJointByName(TEXT("jointBL"));

    // Запуск таймера управления движением
    GetWorld()->GetTimerManager().SetTimer(
        MotionTimerHandle,
        this,
        &UFeedbackMotionComponent::ReadAndApplyMotion,
        0.05f, // 50 мс период
        true
    );
}

void UFeedbackMotionComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (SharedPtr && SharedPtr != MAP_FAILED)
    {
        munmap(SharedPtr, ShmSize);
        SharedPtr = nullptr;
    }

    unlink(TCHAR_TO_ANSI(*ShmPath));
    GetWorld()->GetTimerManager().ClearTimer(MotionTimerHandle);
    Super::EndPlay(EndPlayReason);
}

void UFeedbackMotionComponent::ReadAndApplyMotion()
{
    if (!SharedPtr || SharedPtr == MAP_FAILED)
        return;

    float* Data = static_cast<float*>(SharedPtr);
    float Forward = FMath::Clamp(Data[0], -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
    float Turn    = FMath::Clamp(Data[1], -MAX_TURN_SPEED, MAX_TURN_SPEED);

    // --- Аппроксимация для корректного прямолинейного движения ---
    if (!FMath::IsNearlyZero(Forward))
    {
        FVector ForwardVector = GetOwner()->GetActorForwardVector();
        FVector DesiredVector = FVector(1.0f, 0.0f, 0.0f); // глобальная X-ось
        float AngleOffset = FMath::Acos(FVector::DotProduct(ForwardVector.GetSafeNormal2D(), DesiredVector));

        FVector Cross = FVector::CrossProduct(ForwardVector, DesiredVector);
        float CorrectionSign = (Cross.Z < 0) ? -1.0f : 1.0f;
        float CorrectionTurn = FMath::RadiansToDegrees(AngleOffset) * CorrectionSign * 0.25f;

        Turn += CorrectionTurn; // Корректируем поворот для стабилизации курса
    }

    // Полная остановка — обнуляем усилия
    if (FMath::IsNearlyZero(Forward) && FMath::IsNearlyZero(Turn))
    {
        ApplyMotor(jointFL, 0.0f);
        ApplyMotor(jointFR, 0.0f);
        ApplyMotor(jointBL, 0.0f);
        ApplyMotor(jointBR, 0.0f);
        return;
    }

    // Расчёт крутящих моментов
    float ForwardTorque = Forward * TORQUE_FACTOR_FORWARD;
    float TurnTorque    = Turn    * TORQUE_FACTOR_TURN;

    float LeftTorque  = ForwardTorque - TurnTorque;
    float RightTorque = ForwardTorque + TurnTorque;

    // Применяем к каждому колесу
    ApplyMotor(jointFL, LeftTorque);
    ApplyMotor(jointBL, LeftTorque);
    ApplyMotor(jointFR, RightTorque);
    ApplyMotor(jointBR, RightTorque);
}

void UFeedbackMotionComponent::ApplyMotor(UPhysicsConstraintComponent* Joint, float Speed)
{
    if (!Joint) return;

    float AbsSpeed = FMath::Abs(Speed);
    float Velocity = FMath::Clamp(-Speed / 20.0f, -1.0f, 1.0f); // инвертирован — корректное направление вращения
    float Strength = FMath::Clamp(AbsSpeed * 30.0f, 100.0f, 2000.0f);
    float Damping  = FMath::Clamp(AbsSpeed * 0.4f, 10.0f, 50.0f);

    // Настройка физики для вращения колёс
    Joint->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    Joint->SetAngularVelocityTarget(FVector(0.0f, Velocity, 0.0f)); // Используем ось Y (Swing1)
    Joint->SetAngularDriveParams(0.0f, Strength, Damping);
    Joint->SetAngularVelocityDriveTwistAndSwing(false, true); // Активен только Swing1
}

UPhysicsConstraintComponent* UFeedbackMotionComponent::FindJointByName(FName Name)
{
    AActor* Owner = GetOwner();
    if (!Owner) return nullptr;

    TArray<UActorComponent*> Components;
    Owner->GetComponents(UPhysicsConstraintComponent::StaticClass(), Components);

    for (UActorComponent* Comp : Components)
    {
        if (Comp && Comp->GetFName() == Name)
        {
            return Cast<UPhysicsConstraintComponent>(Comp);
        }
    }

    return nullptr;
}
