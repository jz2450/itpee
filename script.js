/*
  Data fetch script. Uses Fetch to get a text file
  every five seconds, and fill its contents into 
  a div on the HTML page. 
  Based on my fetch example (https://tigoe.github.io/html-for-conndev/fetch/). 
  created 30 Dec 2022
  by Tom Igoe
*/

// this function is called once on page load (see below):
function setup() {
  // set an interval to run fetchText() every 5 seconds:
  setInterval(fetchText, 5000);
}

// make an HTTP call to get a text file:
function fetchText() {
  // parameters for the HTTP/S call
  let params = {
    mode: 'cors', // if you need to turn off CORS, use no-cors
    headers: {    // any HTTP headers you want can go here
      'accept': 'application/text'
    }
  }
  // make the HTTP/S call:
  fetch('log.json', params)
    // .then(response => console.log(response.text))
    .then(response => response.text())  // convert response to text
    // .then(data => getResponse("no error"))
    .then(data => getResponse(data))    // get the body of the response
    .catch(error => getResponse(error));// if there is an error
}

// function to call when you've got something to display:
function getResponse(data) {
  // console.log(data);
  // def needs something to check the data format in case of errors
  // https://bobbyhadz.com/blog/javascript-split-string-by-newline string cleaning tips
  let splitData = data.split(/\r?\n/).filter(Boolean);
  // console.log(splitData);
  getLatest(splitData);
  
}

function getLatest(dataArray) {
  let latestData = dataArray.at(-1);
  latestData = JSON.parse(latestData);
  console.log(latestData);
  let indicator = "❓";
  if (latestData.status == "exited") {
    indicator = "🟢";
  } else if (latestData.status == "entered") {
    indicator = "🔴";
  }
  let secondsSince = parseInt(Date.now()/1000) - latestData["epoch-time"];
  // console.log(latestData);
  let newString = indicator + " North 2 Bathroom - as of " + secondsSince + " seconds ago";
  document.getElementById('result').innerHTML = newString;
}

// This is a listener for the page to load.
// This is the command that actually starts the script:
window.addEventListener('DOMContentLoaded', setup);

