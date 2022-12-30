using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PedestrianController : MonoBehaviour
{
    public float movementSpeed = 1;
    public float rotationSpeed = 120;
    public float stopDistance = 2f;
    public Animator animator;
    public bool reachedDestination;
    public GameObject path;

    private List<Transform> m_nodes = new List<Transform>();
    private int m_currentNodeIndex = 0;
    private Vector3 destination;
    private Vector3 lastPosition;
    private Vector3 velocity;

    // Start is called before the first frame update
    void Start()
    {
        movementSpeed = 1; //Random.Range(0.8f, 2);
        animator = GetComponent<Animator>();

        Transform[] nodeTransforms = path.GetComponentsInChildren<Transform>();
        foreach (Transform trans in nodeTransforms)
        {
            if (trans != path.transform)
            {
                m_nodes.Add(trans);
            }
        }
        destination = m_nodes[m_currentNodeIndex].position;
    }

    // Update is called once per frame
    void Update()
    {
        if(transform.position!= destination)
        {
            Vector3 destinationDirection = destination - transform.position;
            destinationDirection.y = 0;
            float destinationDistance = destinationDirection.magnitude;

            if(destinationDistance >= stopDistance)
            {
                reachedDestination = false;
                Quaternion targetRotation = Quaternion.LookRotation(destinationDirection);
                transform.rotation = Quaternion.RotateTowards(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);
                transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
            }
            else
            {
                reachedDestination = true;
                if(m_currentNodeIndex == m_nodes.Count - 1)
                {
                    m_currentNodeIndex = 0;
                }
                else
                {
                    m_currentNodeIndex++;
                }
                destination = m_nodes[m_currentNodeIndex].position;
            }
        }
        lastPosition = transform.position;
    }

    public void SetDestination(Vector3 destination)
    {
        this.destination = destination;
        reachedDestination = false;
    }
}
