using UnityEngine;

public class AutoRotate : MonoBehaviour
{
    public float rotationSpeed = 30f;

    void Update()
    {
        // Yaw 방향(Y축)으로 자동 회전
        transform.Rotate(Vector3.up * rotationSpeed * Time.deltaTime);
    }
}