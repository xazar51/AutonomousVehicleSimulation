using UnityEngine;

public class PedestrainPathSeg : MonoBehaviour
{
	public bool PedestrainPassing;
	
	void OnTriggerEnter(Collider collider){
		if(collider.tag == "pedestrain"){
			PedestrainPassing = true;
		}
	}
	
	void OnTriggerExit(Collider collider){
		if(collider.tag == "pedestrain"){
			PedestrainPassing = false;
		}
	}
}
