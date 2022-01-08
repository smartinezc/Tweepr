
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
});



// Add keyboard listener to window
window.addEventListener('keydown', moveTweepr);


// Function to move the robot 
function moveTweepr(e)
{
    // Check which key the user pressed
    if(e.code === 'KeyW')
    {
        console.log("Buennas");
    }
    else if(e.code === 'KeyA')
    {
        console.log("Noooo");
    }
    else if(e.code === 'KeyS')
    {
        console.log("SASSSSS");
    }
    else if(e.code === 'KeyD')
    {
        console.log("DDIDIDID");
    }
}