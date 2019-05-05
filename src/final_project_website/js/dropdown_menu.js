/**
Credits for this code go to garfyld on
https://stackoverflow.com/questions/19935480/bootstrap-3-how-to-make-head-of-dropdown-link-clickable-in-navbar
**/

$('li.dropdown :first-child').on('click', function() {
    var $el = $(this).parent();
    if ($el.hasClass('open')) {
        var $a = $el.children('a.dropdown-toggle');
        if ($a.length && $a.attr('href')) {
            location.href = $a.attr('href');
        }
    }
});