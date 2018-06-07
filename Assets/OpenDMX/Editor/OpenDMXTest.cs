using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using OpenDMX;

public class OpenDMXTest {

	[Test]
	public void OpenDMXAllOff() {
		
        OpenDMX.DMX.start();
        
		
		
		for(int i=1;i<=30;i++){
			OpenDMX.DMX.setDmxValue(i,0);
			
		}
		
		OpenDMX.DMX.stop();
		
	}

	
	[Test]
	public void OpenDMXAllOn() {
		
        OpenDMX.DMX.start();
        
		
		
		
		for(int i=1;i<=30;i++){

			OpenDMX.DMX.setDmxValue(i,255);
			
		}
		OpenDMX.DMX.stop();
		
	}

	[UnityTest]
	public IEnumerator OpenDMXRandom() {
		
        OpenDMX.DMX.start();
        
		
		yield return null;
		
		for(int i=1;i<=30;i++){
			byte val = (byte)Random.Range(0,255);
			OpenDMX.DMX.setDmxValue(i,val);
			for(int j=0;j<10;j++)
				yield return null;
		}
		
		OpenDMX.DMX.stop();
	}
}
