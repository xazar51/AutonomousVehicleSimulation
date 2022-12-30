using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Math : MonoBehaviour
{
    public static bool HasPassedNode(Vector3 carPos, Vector3 goingFromPos, Vector3 goingToPos)
    {
        bool hasPassedWaypoint = false;

        //The vector between the character and the waypoint we are going from
        Vector3 a = carPos - goingFromPos;

        //The vector between the waypoints
        Vector3 b = goingToPos - goingFromPos;

        //Vector projection from https://en.wikipedia.org/wiki/Vector_projection
        //To know if we have passed the upcoming waypoint we need to find out how much of b is a1
        //a1 = (a.b / |b|^2) * b
        //a1 = progress * b -> progress = a1 / b -> progress = (a.b / |b|^2)
        float progress = (a.x * b.x + a.y * b.y + a.z * b.z) / (b.x * b.x + b.y * b.y + b.z * b.z);

        //If progress is above 1 we know we have passed the waypoint
        if (progress > 1.0f)
        {
            hasPassedWaypoint = true;
        }

        return hasPassedWaypoint;
    }

    //Should we turn left or right to reach the next waypoint?
    //From: http://www.habrador.com/tutorials/linear-algebra/3-turn-left-or-right/
    public static float SteerDirection(Transform carTrans, Vector3 steerPosition, Vector3 waypointPos)
    {
        //The right direction of the direction you are facing
        Vector3 youDir = carTrans.right;

        //The direction from you to the waypoint
        Vector3 waypointDir = waypointPos - steerPosition;

        //The dot product between the vectors
        float dotProduct = Vector3.Dot(youDir, waypointDir);

        //Now we can decide if we should turn left or right
        float steerDirection = 0f;
        if (dotProduct > 0f)
        {
            steerDirection = 1f;
        }
        else
        {
            steerDirection = -1f;
        }

        return steerDirection;
    }

    
    public static float GetError(Vector3 carPos, Vector3 fromNode, Vector3 toNode)
    {
        //The first part is the same as when we check if we have passed a waypoint

        //The vector between the character and the waypoint we are going from
        Vector3 a = carPos - fromNode;

        //The vector between the waypoints
        Vector3 b = toNode - fromNode;

        //Vector projection from https://en.wikipedia.org/wiki/Vector_projection
        //To know if we have passed the upcoming waypoint we need to find out how much of b is a1
        //a1 = (a.b / |b|^2) * b    
        //a1 = progress * b -> progress = a1 / b -> progress = (a.b / |b|^2)
        Vector3 projection = ((a.x * b.x + a.y * b.y + a.z * b.z) / (b.x * b.x + b.y * b.y + b.z * b.z))*b;

        //The error between the position where the car should be and where it is
        float error = (a - projection).magnitude;
 
        return error;

    }
}