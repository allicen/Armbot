export class ViewService {
    viewToolTip () {
        const codes = document.querySelectorAll('code');
        codes.forEach(function (item) {
            item.addEventListener('click', function () {
                const value = item.innerText;

                // копируем в бокс, потом его удаляем
                const selBox = document.createElement('textarea');
                selBox.style.position = 'fixed';
                selBox.style.left = '0';
                selBox.style.top = '0';
                selBox.style.opacity = '0';
                selBox.value = value;
                document.body.appendChild(selBox);
                selBox.focus();
                selBox.select();
                document.execCommand('copy');
                document.body.removeChild(selBox);

                // подсказка
                const tooltip = document.createElement('i');
                tooltip.innerText = "Скопировано!";
                item.appendChild(tooltip);

                setTimeout(function () {
                    item.removeChild(tooltip);
                }, 500);
            })
        })
    }
}
