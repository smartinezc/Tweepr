
// Obtain the range sliders
const ranges = document.querySelector('.pidGains');
const pRange = document.querySelector('#pRange');
const iRange = document.querySelector('#iRange');
const dRange = document.querySelector('#dRange');

// Obtain the range corresponding textareas
const pVal = document.querySelector('#pVal');
const iVal = document.querySelector('#iVal');
const dVal = document.querySelector('#dVal');


// Add a change listener for range slider
ranges.addEventListener('change', (e) => {
    if(e.target.className === "slider")
    {
        pVal.value = pRange.value;
        iVal.value = iRange.value;
        dVal.value = dRange.value;
    }
    else
    {
        pRange.value = pVal.value;
        iRange.value = iVal.value;
        dRange.value = dVal.value;
    }

    // Send GET request to Tweepr ESP32
    setPIDGains();
});



// Add keyboard listener to window
window.addEventListener('keydown', moveTweepr);


// Function to move the robot 
function moveTweepr(e)
{
    // Check which key the user pressed
    if(e.code === 'KeyW')
    {
        setMove(0.25);
    }
    else if(e.code === 'KeyA')
    {
        setMove(0.25);
    }
    else if(e.code === 'KeyS')
    {
        setMove(-0.25);
    }
    else if(e.code === 'KeyD')
    {
        setMove(-0.25);
    }
}



// Fetch methods to get and post data to Tweepr

// Function to set the new PID Gains values
async function setPIDGains()
{
    const data = await fetch(`http://tweeprcontrol.local/get?pidgains=${pVal.value};${iVal.value};${dVal.value}`);
}

// Function to set the new setPoint
async function setMove(value)
{
    const data = await fetch(`http://tweeprcontrol.local/get?move=${value}`);
    const dataParse = await data.json();
    console.log(dataParse);
}