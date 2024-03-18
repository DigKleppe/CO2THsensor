var data2 = 0;
var simValue1 = 0;
var simValue2 = 0;






// returns array with accumulated momentary values
function getLogMeasValues() {
	var str;
	var req = new XMLHttpRequest();
	req.open("GET", "cgi-bin/getLogMeasValues", false);
	req.send();
	if (req.readyState == 4) {
		if (req.status == 200) {
			str = req.responseText.toString();
			return str;
		//	var arr = str.split(",");
		//	return arr;
		}
	}
}

// returns array minute  averaged values
function getAvgMeasValues() {
	var str;
	var req = new XMLHttpRequest();
	req.open("GET", "cgi-bin/getAvgMeasValues", false);
	req.send();
	if (req.readyState == 4) {
		if (req.status == 200) {
			str = req.responseText.toString();
			return str;
		//	var arr = str.split(",");
		//	return arr;
		}
	}
}


// returns array with momentarty  values
 function getRTMeasValues() {
	var str;
	var req = new XMLHttpRequest();
	req.open("GET", "cgi-bin/getRTMeasValues", false);
	req.send();
	if (req.readyState == 4) {
		if (req.status == 200) {
			str = req.responseText.toString();
			return str;
		//	var arr = str.split(",");
		//	return arr;
		}
	}
}
