document.getElementById('MoodBtn').onclick = function() {
    var div = document.getElementById('ColMood');
    var button = document.getElementById('MoodBtn');
    var o_div = document.getElementById('ColSelTurbine');
    var o_button = document.getElementById('TurbineBtn');
    if (div.hasAttribute('hidden')) {
        div.removeAttribute('hidden');
        button.classList.remove('btn-outline-primary');
        button.classList.add('btn-primary');
        // make sure that the other div is collapsed!
        // make sure that its button is "unactive"
        if (!o_div.hasAttribute('hidden')) {
            o_div.setAttribute('hidden', true);
            o_button.classList.remove('btn-primary');
            o_button.classList.add('btn-outline-primary');
        }
    } else {
        div.setAttribute('hidden', true);
        button.classList.remove('btn-primary');
        button.classList.add('btn-outline-primary');
    }
};

document.getElementById('TurbineBtn').onclick = function() {
    var div = document.getElementById('ColSelTurbine');
    var button = document.getElementById('TurbineBtn');
    var o_div = document.getElementById('ColMood');
    var o_button = document.getElementById('MoodBtn');
    if (div.hasAttribute('hidden')) {
        div.removeAttribute('hidden');
        button.classList.remove('btn-outline-primary');
        button.classList.add('btn-primary');
        // make sure that the other div is collapsed!
        // make sure that its button is "unactive"
        if (!o_div.hasAttribute('hidden')) {
            o_div.setAttribute('hidden', true);
            o_button.classList.remove('btn-primary');
            o_button.classList.add('btn-outline-primary');
        }
    } else {
        div.setAttribute('hidden', true);
        button.classList.remove('btn-primary');
        button.classList.add('btn-outline-primary');
    }
};