using UnityEngine;

public class RandomMover : MonoBehaviour
{
    public float areaSize = 3f;          // The side length of the square area
    public float moveSpeed = 1f;         // Movement speed
    public float updateTargetTime = 2f;  // Time before picking a new point

    private Vector3 targetPos;
    private float timer;

    void Start()
    {
        PickNewTarget();
    }

    void Update()
    {
        // Move towards the target
        transform.position = Vector3.MoveTowards(transform.position, targetPos, moveSpeed * Time.deltaTime);

        // If close to target OR timer runs out — pick new target
        timer -= Time.deltaTime;
        if (Vector3.Distance(transform.position, targetPos) < 0.1f || timer <= 0)
        {
            PickNewTarget();
        }
    }

    void PickNewTarget()
    {
        // 3m x 3m area centered around origin
        float half = areaSize / 2f;

        float x = Random.Range(-half, half);
        float z = Random.Range(-half, half);

        targetPos = new Vector3(x, transform.position.y, z);
        timer = updateTargetTime;
    }
}
